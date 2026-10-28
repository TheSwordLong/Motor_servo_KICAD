"""Conservative sheet-resistance estimate from KiCad filled polygon snapshots.

Run with numpy, scipy, shapely and Pillow installed. This is a DC electrical
model, not a thermal or connector-current-rating certification.
"""
import sys, json, math, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
WORK=ROOT/'tmp/manufacturing_0p5oz_20261028'
sys.path.insert(0,str(WORK/'deps'))
import numpy as np
from scipy import sparse
from scipy.sparse.csgraph import connected_components
from scipy.sparse.linalg import splu
from shapely import Polygon, Point, union_all, make_valid, contains_xy
from PIL import Image, ImageDraw

RHO=1.724e-8*(1+0.00393*40)  # assumed copper temperature 60 C
THICKNESS=15.2e-6
REFS=['CN5','CN6','CN7','CN8']

def verify_uniform_strip():
    # Analytical calibration: 20 by 10 square-cell strip, left edge grounded,
    # 1 A uniformly injected at right edge. Center-to-center length is 19 cells.
    ny,nx=10,20;ids=np.arange(nx*ny).reshape(ny,nx)
    a=np.r_[ids[:,:-1].ravel(),ids[:-1,:].ravel()]
    b=np.r_[ids[:,1:].ravel(),ids[1:,:].ravel()]
    g=THICKNESS/RHO
    lap=sparse.coo_matrix((np.r_[np.full(len(a)*2,g),np.full(len(a)*2,-g)],
                           (np.r_[a,b,a,b],np.r_[a,b,b,a])),shape=(nx*ny,nx*ny)).tocsc()
    free=np.ones(nx*ny,dtype=bool);free[ids[:,0]]=False
    rhs=np.zeros(nx*ny);rhs[ids[:,-1]]=1/ny
    solution=splu(lap[free,:][:,free].tocsc()).solve(rhs[free])
    full=np.zeros(nx*ny);full[free]=solution
    measured=float(full[ids[:,-1]].mean());analytic=RHO/THICKNESS*(nx-1)/ny
    assert abs(measured/analytic-1)<1e-10
    return {'strip_analytical_ohm':analytic,'strip_solver_ohm':measured,'relative_error':abs(measured/analytic-1)}

def geometries(data):
    holes=[]
    for v in data['vias']:
        holes.append(Point(v['position']).buffer(v['drill']/2,quad_segs=24))
    for f in data['footprints']:
        for p in f['pads']:
            if max(p['drill'])>0:
                # Circle circumscribes non-round holes, conservatively omitting
                # additional copper around slots (irrelevant to power terminals).
                holes.append(Point(p['position']).buffer(max(p['drill'])/2,quad_segs=24))
    holes=union_all(holes)
    result=[]
    for p in data['power_planes']:
        raw=[Polygon(c['outer'],c['holes']) for c in p['contours']]
        fixed=union_all([make_valid(r) for r in raw])
        result.append((p['net'],fixed.difference(holes),sum(r.area for r in raw),fixed.area))
    return result

def solve(net,geom,data,h,out):
    start=time.time()
    xmin,ymin,xmax,ymax=geom.bounds
    xs=np.arange(xmin+h/2,xmax,h); ys=np.arange(ymin+h/2,ymax,h)
    xx,yy=np.meshgrid(xs,ys)
    # Keep only cells entirely inside copper. This discards a small edge band,
    # widening clearances and reducing (never adding) modeled copper area.
    safe=geom.buffer(-h/math.sqrt(2))
    mask=contains_xy(safe,xx,yy)
    positions=np.argwhere(mask); n=len(positions)
    ids=np.full(mask.shape,-1,dtype=np.int32); ids[mask]=np.arange(n)
    edgea=np.concatenate([ids[:,:-1][mask[:,:-1]&mask[:,1:]],ids[:-1,:][mask[:-1,:]&mask[1:,:]]])
    edgeb=np.concatenate([ids[:,1:][mask[:,:-1]&mask[:,1:]],ids[1:,:][mask[:-1,:]&mask[1:,:]]])
    cgraph=sparse.coo_matrix((np.ones(len(edgea)*2),(np.r_[edgea,edgeb],np.r_[edgeb,edgea])),shape=(n,n)).tocsr()
    _,component=connected_components(cgraph,directed=False)
    contacts=[]; pads=[]
    for ref in ['CN1']+REFS:
        fp=next(f for f in data['footprints'] if f['ref']==ref)
        pad=next(p for p in fp['pads'] if p['net']==net and max(p['drill'])>0)
        px,py=pad['position']; radius=max(pad['drill'])/2
        # Contact the inner annular band near the plated barrel; do not short
        # the entire terminal pad to a hypothetical perfect conductor.
        contact=np.flatnonzero(mask & ((xx-px)**2+(yy-py)**2 <= (radius+1.8*h)**2))
        assert len(contact)>0,(ref,net,h,'no contact cells')
        contact=ids.ravel()[contact]
        contacts.append(contact); pads.append(pad)
    main=component[contacts[0][0]]
    assert all(np.all(component[c]==main) for c in contacts),'Disconnected power terminal'
    keep=component==main
    # Merge each barrel-contact band to one equipotential electrode.
    parent=np.arange(n)
    for c in contacts: parent[c]=c[0]
    _,reduced=np.unique(parent,return_inverse=True)
    ea=reduced[edgea]; eb=reduced[edgeb]
    valid=(ea!=eb)&keep[edgea]&keep[edgeb]; ea=ea[valid];eb=eb[valid]
    used=np.unique(np.r_[ea,eb]); remap=np.full(reduced.max()+1,-1,dtype=np.int32);remap[used]=np.arange(len(used))
    ea=remap[ea]; eb=remap[eb]; electrodes=[int(remap[reduced[c[0]]]) for c in contacts]
    # Each square cell-to-cell link has conductance sigma*t.
    g=THICKNESS/RHO; nn=len(used)
    lap=sparse.coo_matrix((np.r_[np.full(len(ea)*2,g),np.full(len(ea)*2,-g)],
                          (np.r_[ea,eb,ea,eb],np.r_[ea,eb,eb,ea])),shape=(nn,nn)).tocsc()
    free=np.arange(nn)!=electrodes[0]
    index=np.full(nn,-1,dtype=np.int32);index[free]=np.arange(nn-1)
    mat=lap[free,:][:,free].tocsc()
    rhs=np.zeros((nn-1,4))
    for j,e in enumerate(electrodes[1:]):rhs[index[e],j]=1
    lu=splu(mat); voltage=lu.solve(rhs)
    residual=float(np.max(np.abs(mat@voltage-rhs)))
    assert residual<1e-7, residual
    transfer=voltage[[index[e] for e in electrodes[1:]],:]
    assert np.max(np.abs(transfer-transfer.T))<1e-9
    record={'net':net,'grid_mm':h,'nodes':nn,'discarded_island_cells':int((~keep).sum()),
            'residual_A':residual,'transfer_ohm':transfer.tolist(),
            'R_ohm':np.diag(transfer).tolist(),'contacts':[{'ref':ref,'position':p['position'],'drill_mm':max(p['drill'])} for ref,p in zip(['CN1']+REFS,pads)],
            'elapsed_seconds':round(time.time()-start,2)}
    # Copper-only preview, with terminal centers and reference coordinates.
    scale=8; canvas=Image.new('RGB',(math.ceil((xmax-xmin)*scale)+80,math.ceil((ymax-ymin)*scale)+80),'white')
    raster=Image.fromarray(np.where(mask[...,None],np.array([174,214,180],dtype=np.uint8),np.array([248,248,248],dtype=np.uint8)).astype('uint8'))
    raster=raster.resize((round((xmax-xmin)*scale),round((ymax-ymin)*scale)),Image.Resampling.NEAREST);canvas.paste(raster,(40,40));d=ImageDraw.Draw(canvas)
    for ref,p in zip(['CN1']+REFS,pads):
        x=40+(p['position'][0]-xmin)*scale;y=40+(p['position'][1]-ymin)*scale
        d.ellipse((x-6,y-6,x+6,y+6),outline='red',width=2);d.text((x-10,y-20),ref,fill='black')
    d.text((10,10),f'{net}: actual filled copper, grid {h} mm, inner copper 15.2 um',fill='black')
    canvas.save(out/f'plane_{"24V" if net=="+24V" else "GND"}_{h:g}.png')
    print(json.dumps(record),flush=True)
    return record

def main():
    calibration=verify_uniform_strip()
    data=json.loads((WORK/'after.json').read_text(encoding='utf-8'))
    out=WORK/'power';out.mkdir(exist_ok=True)
    records=[]
    for net,geom,rawarea,validarea in geometries(data):
        assert abs(rawarea-validarea)<.001,(net,rawarea,validarea)
        for h in [.25,.125]:records.append(solve(net,geom,data,h,out))
    mats=[]; convergence=[]
    for net in ['+24V','GND']:
        coarse,fine=[r for r in records if r['net']==net]
        c=np.array(coarse['transfer_ohm']); f=np.array(fine['transfer_ohm'])
        convergence.append({'net':net,'relative_diagonal_difference':(np.abs(np.diag(c)-np.diag(f))/np.diag(f)).tolist()})
        # Take the larger value per entry plus 20% modeling allowance.
        mats.append(np.maximum(c,f)*1.2)
    # Full 1.6 mm barrel length used at each end; 20 um wall copper is an
    # explicit unverified manufacturing assumption. Resin is not conductive.
    common_barrels=2*RHO*.0016/(math.pi*.0016*.000020)
    branch_barrels=2*RHO*.0016/(math.pi*.0014*.000020)
    matrix=mats[0]+mats[1]+common_barrels+np.eye(4)*branch_barrels
    cases=[]
    for i,ref in enumerate(REFS):
        current=np.eye(4)[i]*5
        cases.append({'case':ref+' alone 5 A','current_A':current.tolist(),'drop_V':(matrix@current).tolist(),'active_drop_V':float((matrix@current)[i]),'PCB_loss_W':float(current@matrix@current)})
    current=np.full(4,1.25)
    cases.append({'case':'4 x 1.25 A','current_A':current.tolist(),'drop_V':(matrix@current).tolist(),'PCB_loss_W':float(current@matrix@current)})
    result={'method':'4-neighbor square finite-volume resistor sheet; fully contained copper cells; conservative single ground plane; 20% extra allowance over larger of two mesh results',
            'snapshot_sha256':data['source_sha256'],'copper_temperature_C':60,'rho_ohm_m':RHO,'inner_thickness_um':15.2,
            'barrel_wall_um_assumed':20,'barrel_length_mm_conservative':1.6,'ground_parallel_planes_ignored':True,
            'solver_calibration':calibration,'records':records,'convergence':convergence,'loop_transfer_ohm_including_allowance_and_barrels':matrix.tolist(),
            'cases':cases,'limit_V':.12,'pass_voltage_estimate':bool(np.max(matrix)*5<=.12),
            'thermal_measurement':'NOT PERFORMED; no thermal rise predicted by this DC model',
            'exclusions':['connector contacts','external wiring','contact solder joint defects','dynamic motor regeneration/surges','thermal solution']}
    (out/'power_calculation.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({'pass':result['pass_voltage_estimate'],'cases':cases,'convergence':convergence}),flush=True)
if __name__=='__main__':main()
