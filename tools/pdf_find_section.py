import sys
from pathlib import Path

if len(sys.argv) < 2:
    print('Usage: python pdf_find_section.py <file.pdf>')
    sys.exit(1)

p = Path(sys.argv[1])
data = p.read_bytes()

patterns = [b'4\x00.\x008\x00', b'S\x00e\x00c\x00t\x00i\x00o\x00n\x00 \x004\x00.\x008\x00']
found = False
for pat in patterns:
    idx = data.find(pat)
    if idx != -1:
        found = True
        start = max(0, idx-5000)
        end = min(len(data), idx+5000)
        snippet = data[start:end]
        try:
            text = snippet.decode('utf-16-le', errors='ignore')
            # normalize: remove isolated nulls
            print('--- Decoded UTF-16LE snippet around pattern at byte', idx, '---')
            print(text)
        except Exception as e:
            print('Decode error:', e)
        print()

if not found:
    print('No UTF-16LE pattern found for Section 4.8; attempting ASCII search for "4.8"')
    idx = data.find(b'4.8')
    if idx != -1:
        start = max(0, idx-400)
        end = min(len(data), idx+400)
        print(data[start:end].decode('latin-1', errors='ignore'))
    else:
        print('No ASCII "4.8" found either.')
