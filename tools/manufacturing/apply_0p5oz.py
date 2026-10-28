"""Apply only approved stackup and via manufacturing metadata; preserve geometry."""
import re,sys,hashlib,json
from pathlib import Path
from board_snapshot import children,stable_blocks

ROOT=Path(__file__).resolve().parents[2]
pcb=ROOT/'GD32F303VET6_4Motor_Control.kicad_pcb'
backup=ROOT/'production/backup_before_0p5oz_20261028'/pcb.name
old=pcb.read_text(encoding='utf-8')
assert hashlib.sha256(pcb.read_bytes()).digest()==hashlib.sha256(backup.read_bytes()).digest(),'PCB changed since backup'
general=next(c for c in children(old) if c.startswith('(general'))
setup=next(c for c in children(old) if c.startswith('(setup'))
stack=next(c for c in children(setup) if c.startswith('(stackup'))
newstack=stack
thickness={'F.Cu':.035,'In1.Cu':.0152,'In2.Cu':.0152,'In3.Cu':.0152,'In4.Cu':.0152,'B.Cu':.035,'dielectric 1':.0994,'dielectric 2':.55,'dielectric 3':.1088,'dielectric 4':.55,'dielectric 5':.0994}
materials={'dielectric 1':('FR4 / 3313',4.1),'dielectric 2':('FR4 core TG135',4.6),'dielectric 3':('FR4 / 2116',4.16),'dielectric 4':('FR4 core TG135',4.6),'dielectric 5':('FR4 / 3313',4.1)}
for child in children(stack):
    m=re.match(r'\(layer "([^"]+)"',child)
    if not m:continue
    name=m[1];replacement=child
    if name in thickness:replacement=re.sub(r'\(thickness [^)]+\)',f'(thickness {thickness[name]:g})',replacement)
    if name in materials:
        mat,dk=materials[name]
        replacement=re.sub(r'\(material "[^"]*"\)',f'(material "{mat}")',replacement)
        replacement=re.sub(r'\(epsilon_r [^)]+\)',f'(epsilon_r {dk:g})',replacement)
    if name in ['F.Mask','B.Mask','F.SilkS','B.SilkS']:
        color='Green' if name.endswith('Mask') else 'White'
        replacement=re.sub(r'\s*\(color "[^"]*"\)','',replacement)
        replacement=replacement[:-1]+f'\t(color "{color}")\n\t\t\t)'
    newstack=newstack.replace(child,replacement,1)
newstack=newstack.replace('(copper_finish "None")','(copper_finish "ENIG")')
newsetup=setup.replace(stack,newstack,1).replace('(capping no)','(capping yes)',1).replace('(filling no)','(filling yes)',1)
new=old.replace(general,general.replace('(thickness 1.4)','(thickness 1.6)',1),1).replace(setup,newsetup,1)
assert stable_blocks(old)==stable_blocks(new),'Unexpected geometry/netlist changes'
assert new!=old
pcb.write_text(new,encoding='utf-8',newline='\n')
print('Updated 1.6mm order nominal, physical JLC06161H-3313 layers, ENIG, colors, via filling/capping defaults; geometry unchanged.')
