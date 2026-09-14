# File: tests/run_tests.py
"""Compile real library code against a fake Arduino/Wire API and verify regeneration."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
sys.dont_write_bytecode = True
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from generator import generate


def main():
    """Build both common Wire capacities, run protocol tests and syntax-check examples."""
    compiler = os.environ.get('CXX') or shutil.which('g++')
    if not compiler:
        raise SystemExit('Set CXX to a C++11 compiler')
    schema = json.loads((ROOT / 'tools/commands.json').read_text(encoding='utf-8'))
    with tempfile.TemporaryDirectory(prefix='kinisi-arduino-') as folder:
        out = Path(folder)
        generate(schema, out)
        for name in ['kinisi.h','kinisi.cpp','kinisi_types.h','keywords.txt']:
            assert (out/name).read_bytes() == (ROOT/name).read_bytes(), f'Stale generated file: {name}'
        print('PASS deterministic schema regeneration', flush=True)
        base = [compiler,'-std=c++11','-Wall','-Wextra','-Werror','-I'+str(ROOT/'tests/mocks'),'-I'+str(ROOT)]
        sources = [str(ROOT/p) for p in ['kinisi.cpp','kinisi_protocol.cpp','i2cutils.cpp','tests/test_protocol.cpp']]
        for capacity in [32,64]:
            exe=out/('test'+str(capacity)+('.exe' if os.name=='nt' else ''))
            subprocess.run(base+[f'-DKINISI_WIRE_BUFFER_SIZE={capacity}',*sources,'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True,timeout=20)
        firmware = ROOT.parent / 'kinisi-motor-controller-firmware'
        cc = os.environ.get('CC') or shutil.which('gcc')
        if cc and (firmware/'lib/connection/connection.c').exists():
            includes = [str(firmware/p) for p in ['include','lib/connection','lib/protocol','lib/time_sync','lib/initialization']]
            c_sources = [ROOT/'tests/firmware_session.c'] + [firmware/p for p in ['lib/connection/connection.c','lib/protocol/protocol.c','lib/time_sync/time_sync.c','lib/initialization/initialization.c']]
            objects=[]
            for i, source in enumerate(c_sources):
                obj=out/(str(i)+'.o')
                subprocess.run([cc,'-std=c11','-Wall','-Wextra','-Werror',*['-I'+p for p in includes],'-c',str(source),'-o',str(obj)],check=True)
                objects.append(str(obj))
            exe=out/('interop.exe' if os.name=='nt' else 'interop')
            subprocess.run(base+['-DKINISI_WIRE_BUFFER_SIZE=64',*sources[:-1],str(ROOT/'tests/test_firmware.cpp'),*objects,'-o',str(exe)],check=True)
            subprocess.run([str(exe)],check=True,timeout=20)
        else:
            print('SKIP optional firmware interoperability: requires sibling checkout and CC', flush=True)
        for sketch in (ROOT/'examples').glob('*/*.ino'):
            subprocess.run(base+['-x','c++','-fsyntax-only',str(sketch)],check=True)
        print('PASS example syntax', flush=True)


if __name__ == '__main__':
    main()
