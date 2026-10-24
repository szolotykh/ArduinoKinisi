"""Validate explicit imports and deterministic Arduino profile generation."""
from copy import deepcopy
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


profiles = load('profile_updater', ROOT / 'tools/update-board-profiles.py')
generator = load('generator', ROOT / 'tools/generator.py')
with mock.patch.dict(sys.modules, {'generator': generator}):
    commands = load('command_updater', ROOT / 'tools/update-commands.py')


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.bundle = json.loads((ROOT / 'tools/board_profiles.json').read_text(encoding='utf-8'))

    def test_current_profiles_and_flash_storage(self):
        self.assertEqual(len(self.bundle['boards']), 4)
        for name, content in profiles.generate(self.bundle).items():
            self.assertEqual((ROOT / name).read_text(encoding='utf-8'), content)
        source = profiles.generate(self.bundle)['kinisi_board_profiles.cpp']
        self.assertIn('PROGMEM', source)
        self.assertIn('memcpy_P', source)
        firmware = ROOT.parent / 'kinisi-motor-controller-firmware/include/board_profiles.json'
        if firmware.exists():
            self.assertEqual(self.bundle, json.loads(firmware.read_text(encoding='utf-8')))

    def test_reject_private_and_invalid_profiles(self):
        for mutate in (
            lambda b: b.update(schema_version=2),
            lambda b: b['boards'].append(deepcopy(b['boards'][0])),
            lambda b: b['boards'][0]['board'].update(build_define='MP_V1'),
            lambda b: b['boards'][0]['resources'].update(motors=True),
            lambda b: b['boards'][3]['sensors']['imu'][0].update(id=1),
            lambda b: b['boards'][3]['motor_features'][0].update(current_limit='yes'),
        ):
            bundle = deepcopy(self.bundle); mutate(bundle)
            with self.assertRaises(ValueError):
                profiles.generate(bundle)

    def test_multiple_sensor_metadata_is_generated_from_input(self):
        self.bundle['boards'][3]['sensors']['temperature'].append({'id': 1})
        self.bundle['boards'][3]['sensors']['imu'].append({'id': 1, 'accelerometer': True, 'gyroscope': False})
        generated = profiles.generate(self.bundle)
        self.assertIn('temperature_ids[2]', generated['kinisi_board_profiles.h'])
        self.assertIn('KinisiImuFeatures imu[2]', generated['kinisi_board_profiles.h'])
        self.assertIn('2, {0, 1}, 2, {{0, true, true}, {1, true, false}}', generated['kinisi_board_profiles.cpp'])

    def test_local_import_check_and_rejected_import_preserve_files(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'tools').mkdir()
            source = root / 'input.json'; source.write_text(json.dumps(self.bundle), encoding='utf-8')
            names = profiles.generate(self.bundle).keys()
            with mock.patch.object(profiles, 'ROOT', root), mock.patch.object(profiles, 'urlopen') as fetch, \
                    mock.patch('sys.stdout', new=io.StringIO()):
                profiles.main(['--profiles', str(source)])
                profiles.main(['--profiles', str(source), '--check'])
                fetch.assert_not_called()
                before = {name: (root / name).read_bytes() for name in names}
                self.bundle['schema_version'] = 2
                source.write_text(json.dumps(self.bundle), encoding='utf-8')
                with self.assertRaises(ValueError):
                    profiles.main(['--profiles', str(source)])
                self.assertEqual(before, {name: (root / name).read_bytes() for name in names})

    def test_check_rejects_stale_output_without_replacing_it(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'tools').mkdir()
            source = root / 'input.json'; source.write_text(json.dumps(self.bundle), encoding='utf-8')
            for name, content in profiles.generate(self.bundle).items():
                (root / name).write_text(content, encoding='utf-8')
            (root / 'kinisi_board_profiles.cpp').write_bytes(b'preserved\n')
            with mock.patch.object(profiles, 'ROOT', root), self.assertRaisesRegex(ValueError, 'Stale'):
                profiles.main(['--profiles', str(source), '--check'])
            self.assertEqual((root / 'kinisi_board_profiles.cpp').read_bytes(), b'preserved\n')

    def test_explicit_branch_source(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'tools').mkdir()
            with mock.patch.object(profiles, 'ROOT', root), \
                    mock.patch.object(profiles, 'urlopen', return_value=io.BytesIO(json.dumps(self.bundle).encode())) as fetch, \
                    mock.patch('sys.stdout', new=io.StringIO()):
                profiles.main(['--branch', 'feature/profiles'])
            fetch.assert_called_once_with(
                'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/feature%2Fprofiles/include/board_profiles.json', timeout=30)
            self.assertEqual(json.loads((root / 'tools/board_profiles.json').read_text(encoding='utf-8')), self.bundle)

    def test_sources_must_be_explicit(self):
        for updater in (profiles, commands):
            with mock.patch('sys.stderr', new=io.StringIO()), self.assertRaises(SystemExit):
                updater.main([])

    def test_command_import_preserves_all_files_when_sensors_missing(self):
        original = json.loads((ROOT / 'tools/commands.json').read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); (root / 'tools').mkdir()
            source = root / 'input.json'
            names = ('kinisi.h', 'kinisi.cpp', 'kinisi_types.h', 'keywords.txt', 'tools/commands.json')
            for name in names:
                (root / name).write_bytes(b'preserved\n')
            for missing in ({'GET_TEMPERATURE'}, {'GET_IMU'}, {'GET_TEMPERATURE', 'GET_IMU'}):
                schema = deepcopy(original)
                schema['commands'] = [c for c in schema['commands'] if c['command'] not in missing]
                source.write_text(json.dumps(schema), encoding='utf-8')
                with mock.patch.object(commands, 'ROOT', root), self.assertRaisesRegex(ValueError, 'missing runtime commands'):
                    commands.main(['--schema', str(source)])
                for name in names:
                    self.assertEqual((root / name).read_bytes(), b'preserved\n')


if __name__ == '__main__':
    unittest.main()
