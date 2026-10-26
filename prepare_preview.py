from pathlib import Path
import gzip,base64,json
R=Path(__file__).resolve().parent
scene=json.loads((R/'scene.json').read_text(encoding='utf-8'))
for geometry in scene['geometries'].values():
    geometry['v']=[round(x,1) for x in geometry['v']]
data=json.dumps(scene,separators=(',',':')).encode()
template=(R/'viewer-template.html').read_text(encoding='utf-8')
payload=json.dumps({'gzip':base64.b64encode(gzip.compress(data,compresslevel=9)).decode()})
fragment=template.replace('__MODEL_DATA__',payload)
(R/'microduck-original-look.html').write_text(fragment,encoding='utf-8')
assert (R/'microduck-original-look.html').stat().st_size<1000000,'Inline preview exceeds 1 MB'
head=(R/'preview-header.html').read_text(encoding='utf-8')
tail='<div class="note text-small">外形统一放大 2 倍，当前站姿约高 54.3 cm；舵机与新夹座按实际尺寸建模。已替换原 XL330 显示模型，旧内部连接件仅作位置参考，尚未适配完成。不是可直接整机打印、装配和行走的定稿。</div></main></body></html>'
(R/'preview.html').write_text(head+fragment+tail,encoding='utf-8')
print(json.dumps({'fragment_bytes':(R/'microduck-original-look.html').stat().st_size}))
