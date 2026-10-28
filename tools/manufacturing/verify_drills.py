"""Compare Excellon holes/slots to actual KiCad pads/vias (KiCad Python)."""
import json,math,re
from pathlib import Path
import pcbnew
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/'tmp/manufacturing_0p5oz_20261028'
b=pcbnew.LoadBoard(str(ROOT/'GD32F303VET6_4Motor_Control.kicad_pcb'))
expected={'PTH':[],'NPTH':[]}
mm=pcbnew.ToMM
for v in b.GetTracks():
    if isinstance(v,pcbnew.PCB_VIA):
        expected['PTH'].append([mm(v.GetPosition().x),-mm(v.GetPosition().y),mm(v.GetDrill()),0,0])
        assert v.GetPrimaryDrillFilledFlag() and v.GetPrimaryDrillCappedFlag()
for f in b.GetFootprints():
    for p in f.Pads():
        w,h=mm(p.GetDrillSize().x),mm(p.GetDrillSize().y)
        if min(w,h)<=0:continue
        theta=math.radians(p.GetOrientation().AsDegrees())
        # Absolute projections of the slot centerline; both endpoints symmetric.
        dx,dy=(abs(w-h)*math.cos(theta),abs(w-h)*math.sin(theta)) if w>=h else (abs(w-h)*math.sin(theta),abs(w-h)*math.cos(theta))
        kind='NPTH' if p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH else 'PTH'
        expected[kind].append([mm(p.GetPosition().x),-mm(p.GetPosition().y),min(w,h),abs(dx),abs(dy)])
results={}
for kind in ['PTH','NPTH']:
    text=next((WORK/'drill').glob('*-'+kind+'.drl')).read_text()
    assert ('NonPlated' if kind=='NPTH' else 'Plated')+',1,6' in text
    assert 'absolute / metric / decimal' in text and 'G90' in text
    tools={};actual=[];tool=None
    for line in text.splitlines():
        m=re.fullmatch(r'T(\d+)C([\d.]+)',line)
        if m:tools[m[1]]=float(m[2]);continue
        m=re.fullmatch(r'T(\d+)',line)
        if m:tool=m[1];continue
        if line.startswith('X'):
            coords=[float(v) for v in re.findall(r'[XY](-?[\d.]+)',line)]
            if len(coords)==2:actual.append(coords+[tools[tool],0,0])
            else:
                x,y,x2,y2=coords
                actual.append([(x+x2)/2,(y+y2)/2,tools[tool],abs(x-x2),abs(y-y2)])
    assert len(actual)==len(expected[kind]),(kind,len(actual),len(expected[kind]))
    remaining=list(expected[kind]);max_error=0
    for hole in actual:
        best=min(range(len(remaining)),key=lambda i:max(abs(a-b) for a,b in zip(hole,remaining[i])))
        error=max(abs(a-b) for a,b in zip(hole,remaining.pop(best)))
        assert error<=.00101,(kind,hole,error)
        max_error=max(max_error,error)
    results[kind]={'hole_count':len(actual),'slots':sum(x[3]+x[4]>.0001 for x in actual),'max_coordinate_or_size_rounding_error_mm':max_error,'all_coordinates_diameters_slot_axes_and_plating_match':True}
results['explicit_filled_and_capped_vias']=250
(WORK/'drill_validation.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results))
