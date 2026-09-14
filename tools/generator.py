# File: tools/generator.py
"""Generate Arduino API-v2 declarations and packed little-endian command codecs."""
import argparse
import json
from pathlib import Path

SIZES = {'bool': 1, 'uint8_t': 1, 'int8_t': 1, 'uint16_t': 2, 'int16_t': 2,
         'uint32_t': 4, 'int32_t': 4, 'uint64_t': 8, 'int64_t': 8, 'double': 8}
INTERNAL = {'INIT', 'TIME_SYNC_RESPONSE'}
HEADER = '// Generated from tools/commands.json; edit the schema or generator instead.\n'


def public_commands(schema):
    """Select user requests; connection setup and clock replies belong to the runtime."""
    return [c for c in schema['commands']
            if c['direction'] == 'client_to_controller' and c['command'] not in INTERNAL]


def response_type(command):
    """Resolve both legacy object references and directly named response types."""
    r = command.get('response')
    return (r['name'] if r['type'] == 'object' else r['type']) if r else None


def generate(schema, output):
    """Validate schema types and emit deterministic headers, methods and Arduino keywords."""
    if schema['version'] != '2.0.0':
        raise ValueError('This runtime supports protocol 2.0.0')
    objects = {o['name']: o for o in schema['objects']}
    for c in schema['commands']:
        if c.get('direction') not in ('client_to_controller', 'controller_to_client'):
            raise ValueError('Missing or invalid direction: ' + c['command'])
    def size(t):
        return SIZES[t] if t in SIZES else sum(size(p['type']) for p in objects[t]['properties'])
    def decode(t, expression):
        return f'kinisi_codec::readDouble({expression})' if t == 'double' else f'static_cast<{t}>(kinisi_codec::readUnsigned({expression}, {size(t)}))'
    types = [HEADER, '#ifndef KINISI_TYPES_H\n#define KINISI_TYPES_H\n#include <stdint.h>\n']
    for c in schema['commands']:
        types.append(f"static const uint8_t KINISI_{c['command']} = {c['code']};\n")
    types.append('\n/** Shared controller error codes; see commands.json for per-command errors. */\nenum class KinisiErrorCode : uint8_t {\n    NONE = 0,\n')
    for e in schema['error_codes']:
        types.append(f"    {e['name']} = {e['code']}, // {e['description']}\n")
    types.append('};\n')
    for o in objects.values():
        types.append(f"\n/** {o['description']} */\nstruct {o['name']} {{\n")
        for p in o['properties']:
            types.append(f"    {p['type']} {p['name']}; ///< {p['description']}\n")
        types.append('};\n')
    types.append('#endif\n')
    header = [HEADER, '#ifndef KINISI_H\n#define KINISI_H\n#include "kinisi_protocol.h"\n\n',
              '/** Blocking I2C client. begin() completes API-v2 INIT and READY in uptime mode. */\n',
              'class KinisiController : public KinisiProtocol {\npublic:\n',
              '    /** Select a seven-bit I2C address (default 8). Does not touch the bus. */\n',
              '    explicit KinisiController(uint8_t address = 8) : KinisiProtocol(address) {}\n']
    source = [HEADER, '#include "kinisi.h"\n#include "kinisi_codec.h"\n']
    for c in public_commands(schema):
        props = c.get('properties', [])
        args = ', '.join(p['type'] + ' ' + p['name'] for p in props)
        rt = response_type(c)
        method = c['command'].lower()
        doc = c['description'] + ' Errors: ' + ', '.join(c.get('errors', [])) + '.'
        header.append(f'\n    /** {doc} Check lastError() after getters; setters return success. */\n    {rt or "bool"} {method}({args});\n')
        source.append(f'\n/** Encode {c["command"]}, match its reply, and decode the payload. */\n{rt or "bool"} KinisiController::{method}({args}) {{\n')
        n = sum(size(p['type']) for p in props)
        source.append(f'    uint8_t payload[{max(n, 1)}] = {{0}};\n')
        offset = 0
        for p in props:
            target = f'payload + {offset}'
            if p['type'] == 'double':
                source.append(f'    kinisi_codec::writeDouble({target}, {p["name"]});\n')
            else:
                source.append(f'    kinisi_codec::writeUnsigned({target}, static_cast<uint64_t>({p["name"]}), {size(p["type"])});\n')
            offset += size(p['type'])
        if rt:
            source.append(f'    {rt} result = {{}};\n    uint8_t response[{size(rt)}] = {{0}};\n')
            source.append(f'    if (!request(KINISI_{c["command"]}, payload, {n}, response, {size(rt)})) return result;\n')
            if rt in objects:
                offset = 0
                for p in objects[rt]['properties']:
                    source.append(f'    result.{p["name"]} = {decode(p["type"], "response + " + str(offset))};\n')
                    offset += size(p['type'])
            else:
                source.append(f'    result = {decode(rt, "response")};\n')
            source.append('    return result;\n}\n')
        else:
            source.append(f'    return request(KINISI_{c["command"]}, payload, {n}, nullptr, 0);\n}}\n')
    header.append('};\n#endif\n')
    keywords = '# Generated Arduino syntax map.\nKinisiController\tKEYWORD1\nbegin\tKEYWORD2\nready\tKEYWORD2\nboardInfo\tKEYWORD2\nlastError\tKEYWORD2\n'
    keywords += ''.join(c['command'].lower() + '\tKEYWORD2\n' for c in public_commands(schema))
    output.mkdir(parents=True, exist_ok=True)
    for name, content in [('kinisi_types.h', ''.join(types)), ('kinisi.h', ''.join(header)), ('kinisi.cpp', ''.join(source)), ('keywords.txt', keywords)]:
        (output / name).write_text(content, encoding='utf-8', newline='\n')


def main():
    """Read an explicitly selected schema and generate files in the requested directory."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_json_path', type=Path)
    parser.add_argument('output_path', type=Path)
    args = parser.parse_args()
    generate(json.loads(args.input_json_path.read_text(encoding='utf-8')), args.output_path)


if __name__ == '__main__':
    main()
