# Robot CAD -> light STL shells, with FreeCAD's own kernel (no third-party CAD libraries).
#
#   "C:\Program Files\FreeCAD 1.1\bin\FreeCADCmd.exe" tools/step_to_stl.py -- SRC DST [FACETS]
#
# SRC is a folder of STEP parts (a maker's "3D data divided by axis" download), DST the folder to
# write <name>.stl into, FACETS the triangle budget per part (default 4000).
#
# Why a budget: the viewer draws a robot every frame, and a maker's STEP tessellates to 10-40k
# triangles per link - 130k for one arm, 6 MB of STL. Decimated to 4k a link the silhouette is the
# same at cell scale and the whole arm is about 20k triangles, which is what the rest of a scene
# costs. A shell is DRAWN and never collided (CLAUDE.md), so nothing about the physics depends on
# how fine it is.
#
# The vendor CAD itself is NOT redistributable: keep both the STEP and the STL out of git (see
# .gitignore) and let the scene fall back to its primitives when the files are not there.
import os
import sys
import glob

import Part
import Mesh
import MeshPart

src = os.environ.get('MIO_SRC', '.')
dst = os.environ.get('MIO_DST', 'out')
budget = int(os.environ.get('MIO_FACETS', '4000'))

os.makedirs(dst, exist_ok=True)
log = []
for f in sorted(glob.glob(os.path.join(src, '*.stp')) + glob.glob(os.path.join(src, '*.step'))):
    name = os.path.splitext(os.path.basename(f))[0]
    # A maker's part file is named for the whole order code; keep the last -cNNN as the link name.
    tail = name.split('-')[-1]
    if tail.startswith('c') and tail[1:].isdigit():
        name = 'j' + str(int(tail[1:]))
    elif name.endswith('_asm'):
        continue                                   # the assembly repeats every part
    shape = Part.Shape()
    shape.read(f)
    mesh = MeshPart.meshFromShape(Shape=shape, LinearDeflection=4.0, AngularDeflection=0.6, Relative=False)
    before = mesh.CountFacets
    if before > budget:
        # decimate(tolerance mm, reduction 0..1): a 2 mm tolerance keeps the silhouette of a casting
        # whose smallest features are bolt heads.
        mesh.decimate(2.0, 1.0 - float(budget) / before)
    out = os.path.join(dst, name + '.stl')
    mesh.write(out)
    bb = shape.BoundBox
    log.append('%-4s %6d -> %5d tris  %4d KB  bbox x[%.0f %.0f] y[%.0f %.0f] z[%.0f %.0f]' % (
        name, before, mesh.CountFacets, os.path.getsize(out) / 1024,
        bb.XMin, bb.XMax, bb.YMin, bb.YMax, bb.ZMin, bb.ZMax))

sys.stderr.write('\n'.join(log) + '\n')
