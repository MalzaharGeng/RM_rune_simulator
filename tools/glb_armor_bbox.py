"""从 vehicle.glb 直接读出 ARMOR_*_P 的局部包围盒，确认装甲板真实尺寸。"""
from pathlib import Path
import json
import struct
import sys

PATH = str(Path(__file__).resolve().parents[1] / "src" / "assets" / "vehicle.glb")

with open(PATH, "rb") as f:
    data = f.read()

magic, ver, length = struct.unpack_from("<III", data, 0)
assert magic == 0x46546C67, "not a glb"
off = 12
js = None
while off < length:
    clen, ctype = struct.unpack_from("<II", data, off)
    chunk = data[off + 8 : off + 8 + clen]
    if ctype == 0x4E4F534A:
        js = json.loads(chunk.decode("utf-8"))
    off += 8 + clen

print("glTF version", ver, " nodes:", len(js.get("nodes", [])), " meshes:", len(js.get("meshes", [])))
print()

accessors = js["accessors"]
nodes = js["nodes"]
meshes = js["meshes"]


def node_bbox(ni, depth=0):
    n = nodes[ni]
    out = []
    if "mesh" in n:
        for prim in meshes[n["mesh"]]["primitives"]:
            acc = accessors[prim["attributes"]["POSITION"]]
            if "min" in acc and "max" in acc:
                out.append((acc["min"], acc["max"]))
    return out


for i, n in enumerate(nodes):
    name = n.get("name", "")
    if name.startswith("ARMOR_") or "ARMOR" in name:
        bb = node_bbox(i)
        if bb:
            for mn, mx in bb:
                size = [round(mx[k] - mn[k], 5) for k in range(3)]
                print(f"[{i:>4}] {name:<24} size(XYZ) = {size}   min={[round(v,4) for v in mn]} max={[round(v,4) for v in mx]}")
        else:
            print(f"[{i:>4}] {name:<24} (no mesh / no bbox)")
