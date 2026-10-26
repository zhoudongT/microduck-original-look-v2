"""Verify that exported exterior studies preserve the exact original outline at 2x."""
from pathlib import Path
import sys,json,numpy as np
R=Path(__file__).resolve().parent
import trimesh
from scipy.spatial import cKDTree
rows=[]
for path in sorted((R/'cad/exterior-study-stl').glob('*.stl')):
    name=path.stem.replace('_2x_REFERENCE','')
    original=trimesh.load_mesh(R/'source/official/assets'/f'{name}.stl',process=True)
    expected=original.vertices*2000 # source metres -> mm; 2x scale
    exported=trimesh.load_mesh(path,process=True)
    deviation=max(cKDTree(expected).query(exported.vertices)[0].max(),cKDTree(exported.vertices).query(expected)[0].max())
    passed=bool(deviation<.001 and len(original.faces)==len(exported.faces) and exported.is_watertight)
    rows.append(dict(part=name,scale=2,maximum_vertex_deviation_mm=float(deviation),same_triangle_count=len(original.faces)==len(exported.faces),watertight=bool(exported.is_watertight),passed=passed))
report=dict(scope='17 exported exterior meshes vs original vertices and topology after uniform 2x scale; does not validate assembly/manufacturing',passed=len(rows)==17 and all(r['passed'] for r in rows),parts=rows)
(R/'exterior-fidelity-report.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(dict(parts=len(rows),passed=report['passed'],max_deviation_mm=max(r['maximum_vertex_deviation_mm'] for r in rows))))
sys.exit(0 if report['passed'] else 1)
