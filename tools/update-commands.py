# File: tools/update-commands.py
"""Regenerate the library from an explicit local schema or firmware Git branch."""
import argparse
import json
from pathlib import Path
from urllib.request import urlopen
from urllib.parse import quote
from generator import generate


def main():
    """Load and validate the schema before replacing the checked-in snapshot."""
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--schema', type=Path)
    source.add_argument('--branch')
    args = parser.parse_args()
    if args.schema:
        raw = args.schema.read_text(encoding='utf-8')
    else:
        url = 'https://raw.githubusercontent.com/szolotykh/kinisi-motor-controller-firmware/' + quote(args.branch, safe='') + '/commands.json'
        with urlopen(url, timeout=30) as response:
            raw = response.read().decode('utf-8')
    schema = json.loads(raw)
    root = Path(__file__).resolve().parents[1]
    generate(schema, root)
    (root / 'tools/commands.json').write_text(raw, encoding='utf-8', newline='\n')


if __name__ == '__main__':
    main()
