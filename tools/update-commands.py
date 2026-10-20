# File: tools/update-commands.py
"""Regenerate the library from an explicit local schema or firmware Git branch."""
import argparse
import json
from pathlib import Path
from urllib.request import urlopen
from urllib.parse import quote
from generator import generate

ROOT = Path(__file__).resolve().parents[1]

def main(argv=None):
    """Load and validate the schema before replacing the checked-in snapshot."""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--schema', type=Path)
    source.add_argument('--branch')
    parser.add_argument('--check', action='store_true', help='Check bundled output without writing')
    args = parser.parse_args(argv)
    if args.schema:
        raw = args.schema.read_text(encoding='utf-8')
    else:
        url = 'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/' + quote(args.branch, safe='') + '/commands.json'
        with urlopen(url, timeout=30) as response:
            raw = response.read().decode('utf-8')
    schema = json.loads(raw)
    generate(schema, ROOT, check=args.check)
    snapshot = ROOT / 'tools/commands.json'
    if args.check:
        if not snapshot.exists() or json.loads(snapshot.read_text(encoding='utf-8')) != schema:
            raise ValueError('Bundled command schema is stale')
    else:
        snapshot.write_text(raw, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
