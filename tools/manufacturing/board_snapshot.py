"""Read-only KiCad geometry export for manufacturing checks. Run with KiCad Python."""
import json, sys, hashlib, re
from pathlib import Path
import pcbnew

def mm(v): return pcbnew.ToMM(v)
def xy(p): return [mm(p.x), mm(p.y)]
def children(s):
    depth=0; start=0; quoted=False; escape=False
    for i,c in enumerate(s):
        if quoted:
            if escape: escape=False
            elif c=='\\': escape=True
            elif c=='"': quoted=False
        elif c=='"': quoted=True
        elif c=='(':
            depth+=1
            if depth==2:start=i
        elif c==')':
            if depth==2:yield s[start:i+1]
            depth-=1
    assert depth==0
def stable_blocks(text):
    result=[]
    for child in children(text):
        head=child.split(None,1)[0]
        if head in ['(setup','(general']:continue
        if head=='(zone':
            child='(zone '+' '.join(c for c in children(child) if not c.startswith('(filled_polygon') and not c.startswith('(filled_areas_thickness'))+')'
        if head=='(via':
            child='(via '+' '.join(c for c in children(child) if not c.startswith('(capping') and not c.startswith('(filling'))+')'
        tokens=re.findall(r'"(?:\\.|[^"\\])*"|[()]|[^\s()]+',child)
        result.append(' '.join(tokens))
    return sorted(result)
def run(pcb_path,output):
    b=pcbnew.LoadBoard(str(pcb_path))
    data={'source_sha256':hashlib.sha256(pcb_path.read_bytes()).hexdigest(),'thickness_mm':mm(b.GetDesignSettings().GetBoardThickness()),'copper_layers':b.GetCopperLayerCount(),'footprints':[], 'vias':[], 'power_planes':[]}
    for f in b.GetFootprints():
        data['footprints'].append({'ref':f.GetReference(),'value':f.GetValue(),'footprint':str(f.GetFPID().GetLibNickname())+':'+str(f.GetFPID().GetLibItemName()),'dnp':f.IsDNP(),'exclude_bom':f.IsExcludedFromBOM(),'position':xy(f.GetPosition()),'rotation':f.GetOrientationDegrees(),'side':'Bottom' if f.IsFlipped() else 'Top','pads':[{'num':p.GetNumber(),'net':p.GetNetname(),'position':xy(p.GetPosition()),'size':xy(p.GetSize()),'drill':xy(p.GetDrillSize()),'npth':p.GetAttribute()==pcbnew.PAD_ATTRIB_NPTH} for p in f.Pads()]})
    for t in b.GetTracks():
        if isinstance(t,pcbnew.PCB_VIA):data['vias'].append({'position':xy(t.GetPosition()),'net':t.GetNetname(),'drill':mm(t.GetDrill()),'diameter':mm(t.GetWidth(pcbnew.F_Cu)),'filling':int(t.GetFillingMode()),'capping':int(t.GetCappingMode())})
    for net,layer in [('+24V',pcbnew.In2_Cu),('GND',pcbnew.In1_Cu)]:
        contours=[]
        for z in b.Zones():
            if z.GetNetname()!=net or not z.IsOnLayer(layer):continue
            poly=z.GetFilledPolysList(layer)
            for i in range(poly.OutlineCount()):
                ring=poly.Outline(i)
                contours.append({'outer':[xy(ring.CPoint(j)) for j in range(ring.PointCount())], 'holes':[[xy(poly.Hole(i,h).CPoint(j)) for j in range(poly.Hole(i,h).PointCount())] for h in range(poly.HoleCount(i))]})
        data['power_planes'].append({'net':net,'layer':b.GetLayerName(layer),'contours':contours})
    # All drill holes remove planar copper, including same-net plated bores.
    data['stable_blocks']=stable_blocks(pcb_path.read_text(encoding='utf-8'))
    output.write_text(json.dumps(data,ensure_ascii=False),encoding='utf-8')
    print(json.dumps({'footprints':len(data['footprints']),'vias':len(data['vias']),'power_plane_vertices':[sum(len(c['outer']) for c in p['contours']) for p in data['power_planes']]},ensure_ascii=False))
if __name__=='__main__':run(Path(sys.argv[1]),Path(sys.argv[2]))
