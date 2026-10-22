"""Read-only UTF-8, links, fences and complete RTU example checks. No hardware."""
from pathlib import Path
import re
from urllib.parse import unquote
import importlib.util
root=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('host',root/'tools/robot_host.py')
host=importlib.util.module_from_spec(spec);spec.loader.exec_module(host)
files=[root/'README.md',*sorted((root/'docs').glob('Robot_*.md'))]
links=frames=0
errors=[]
for p in files:
 s=p.read_text(encoding='utf-8')
 fences=sum(line.lstrip().startswith('```') for line in s.splitlines())
 if fences%2:errors.append(f'{p.name}: unclosed fence')
 for target in re.findall(r'\]\(([^)]+)\)',s):
  if '://' in target or target.startswith('mailto:'):continue
  target=target.strip('<>');path,_,anchor=target.partition('#')
  q=(p.parent/unquote(path)) if path else p
  if not q.exists():errors.append(f'{p.name}: missing link {target}')
  elif anchor and q.suffix=='.md':
   text=q.read_text(encoding='utf-8')
   anchors=set(re.findall(r'<a\s+(?:id|name)="([^"]+)"',text))
   for heading in re.findall(r'^#{1,6}\s+(.+)$',text,re.M):
    heading=re.sub(r'[^\w\-\s]','',heading.lower()).replace(' ','-')
    anchors.add(heading)
   if unquote(anchor) not in anchors:errors.append(f'{p.name}: missing anchor {target}')
  links+=1
 for match in re.finditer(r'(?<![0-9A-F])01(?: [0-9A-F]{2}){4,}(?![0-9A-F])',s):
  b=bytes.fromhex(match.group());f=b[1]
  is_rtu=(f==3 and ((len(b)==8 and 1<=int.from_bytes(b[4:6],'big')<=125) or len(b)==5+b[2])) or (f==6 and len(b)==8) or (f==16 and (len(b)==8 or (len(b)>=9 and len(b)==9+b[6]))) or (f in (0x83,0x86,0x90) and len(b)==5)
  # CAN type-01 read: tag/address/count followed by three reserved zeros.
  if len(b)==8 and b[4] in (1,2) and b[5:]==bytes(3):
   is_rtu=False
  if is_rtu:
   if host.crc16(b):errors.append(f'{p.name}: bad CRC {match.group()}')
   frames+=1
assert not errors,'\n'.join(errors)
print(f'PASS: {len(files)} UTF-8 documents, {links} local links/anchors, {frames} complete RTU examples; fences balanced.')
