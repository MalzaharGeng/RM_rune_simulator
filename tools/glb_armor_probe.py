# -*- coding: utf-8 -*-
"""从 vehicle.glb 提取 ARMOR_*_P / ARMOR_*_L 网格在 BASE 坐标系下的质心与包围盒。"""
from pathlib import Path
import json, struct, sys

P = str(Path(__file__).resolve().parents[1] / "src" / "assets" / "vehicle.glb")
data = open(P, 'rb').read()
magic, ver, length = struct.unpack_from('<III', data, 0)
off, gltf, bin_chunk = 12, None, None
while off < len(data):
    clen, ctype = struct.unpack_from('<II', data, off); off += 8
    chunk = data[off:off+clen]; off += clen
    if ctype == 0x4E4F534A: gltf = json.loads(chunk.decode('utf-8'))
    elif ctype == 0x004E4942: bin_chunk = chunk

nodes, meshes, accs, bvs = gltf['nodes'], gltf['meshes'], gltf['accessors'], gltf['bufferViews']
CT = {5120:('b',1),5121:('B',1),5122:('h',2),5123:('H',2),5125:('I',4),5126:('f',4)}
NC = {'SCALAR':1,'VEC2':2,'VEC3':3,'VEC4':4,'MAT4':16}

def read_vec3(acc_idx):
    a = accs[acc_idx]
    assert a['type'] == 'VEC3' and a['componentType'] == 5126, a
    bv = bvs[a['bufferView']]
    base = bv.get('byteOffset', 0) + a.get('byteOffset', 0)
    stride = bv.get('byteStride') or 12
    out = []
    for i in range(a['count']):
        x, y, z = struct.unpack_from('<fff', bin_chunk, base + i*stride)
        out.append((x, y, z))
    return out

# 建立 父->子 映射，用于计算节点到 BASE 的累积变换 (这里只处理平移, 已知旋转/缩放≈单位)
parent = {}
for i, n in enumerate(nodes):
    for c in n.get('children', []):
        parent[c] = i

def chain_t(i):
    """返回从 BASE 到节点 i 的平移累积 (不含旋转)。"""
    t = [0.0, 0.0, 0.0]
    cur = i
    while cur is not None:
        n = nodes[cur]
        tr = n.get('translation')
        if tr: t = [t[k] + tr[k] for k in range(3)]
        cur = parent.get(cur)
    return t

print("%-14s %-34s %-34s" % ("node", "平移链(累积, BASE系)", "网格质心+平移 = BASE系位置"))
print("-" * 90)
for i, n in enumerate(nodes):
    nm = n.get('name', '?')
    if not (nm.startswith('ARMOR_') and (nm.endswith('_P') or nm.endswith('_L'))):
        continue
    t = chain_t(i)
    m = n.get('mesh')
    if m is None:
        print("%-14s %-34s (no mesh)" % (nm, t)); continue
    prim = meshes[m]['primitives'][0]
    pos = read_vec3(prim['attributes']['POSITION'])
    cx = sum(p[0] for p in pos)/len(pos)
    cy = sum(p[1] for p in pos)/len(pos)
    cz = sum(p[2] for p in pos)/len(pos)
    mn = [min(p[k] for p in pos) for k in range(3)]
    mx = [max(p[k] for p in pos) for k in range(3)]
    ctr = (cx+t[0], cy+t[1], cz+t[2])
    print("%-14s T=%-30s C=(%+.4f,%+.4f,%+.4f)  AABB=[%.4f..%.4f, %.4f..%.4f, %.4f..%.4f] n=%d"
          % (nm, "[%+.4f,%+.4f,%+.4f]" % tuple(t), ctr[0], ctr[1], ctr[2],
             mn[0], mx[0], mn[1], mx[1], mn[2], mx[2], len(pos)))
