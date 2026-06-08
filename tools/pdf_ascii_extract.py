import sys
from pathlib import Path

if len(sys.argv) < 2:
    print('Usage: python pdf_ascii_extract.py <file.pdf>')
    sys.exit(1)

p = Path(sys.argv[1])
if not p.exists():
    print('File not found:', p)
    sys.exit(2)

data = p.read_bytes()
# extract runs of printable ASCII
runs = []
current = []
for b in data:
    if 32 <= b <= 126 or b in (9,10,13):
        current.append(chr(b))
    else:
        if len(current) >= 5:
            runs.append(''.join(current))
        current = []
if len(current) >= 5:
    runs.append(''.join(current))

text = '\n'.join(runs)
# find occurrences of '4.8' or 'Section 4.8'
for i, line in enumerate(text.splitlines()):
    if '4.8' in line or 'Section 4.8' in line or '4. 8' in line:
        start = max(0, i-3)
        end = min(i+4, len(text.splitlines()))
        print('--- Match at line', i, '---')
        for l in text.splitlines()[start:end]:
            print(l)
        print()

# If nothing found, print an excerpt around where '4' appears near '8'
if '4.8' not in text:
    # print a search for chapter 4 headings
    for i, line in enumerate(text.splitlines()):
        if line.strip().startswith('Chapter 4') or ('4.' in line and 'Chapter' in line):
            start = max(0, i-3)
            end = min(i+10, len(text.splitlines()))
            print('--- Chapter 4 context at', i, '---')
            for l in text.splitlines()[start:end]:
                print(l)
            print()

# Also dump first 2000 chars in case
print('\n--- BEGIN ASCII DUMP (first 2000 chars) ---')
print(text[:2000])
print('--- END DUMP ---')
