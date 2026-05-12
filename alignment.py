import slicer
import vtk
import numpy as np

# ==================================================
# PARAMETERS
# ==================================================

DS = 1.0
EPS = 1e-6

# If catheter_1[0] is already the base, set True.
# If catheter_1[0] is the tip, set False.
CATheter_FIRST_POINT_IS_BASE = False

# ==================================================
# NODES
# ==================================================

needleNode = slicer.util.getNode("NeedleFiducials")  # always base -> tip
catNode    = slicer.util.getNode("catheter_1")

# ==================================================
# UTILITIES
# ==================================================

def get_pts(node):
    pts = []
    n = node.GetNumberOfControlPoints()
    for i in range(n):
        if not node.GetNthControlPointVisibility(i):
            continue
        p = [0.0, 0.0, 0.0]
        node.GetNthControlPointPositionWorld(i, p)
        p = np.array(p, dtype=float)
        if np.all(np.isfinite(p)):
            pts.append(p)
    pts = np.asarray(pts, dtype=float)
    if len(pts) < 2:
        raise RuntimeError("Need at least 2 valid control points.")
    return pts

def resample(points, ds):
    seg = np.linalg.norm(np.diff(points, axis=0), axis=1)
    arc = np.insert(np.cumsum(seg), 0, 0.0)
    if arc[-1] < EPS:
        raise RuntimeError("Curve length too small.")

    s = np.arange(0.0, arc[-1], ds)
    if len(s) == 0 or abs(s[-1] - arc[-1]) > EPS:
        s = np.append(s, arc[-1])

    return np.vstack([
        np.interp(s, arc, points[:, 0]),
        np.interp(s, arc, points[:, 1]),
        np.interp(s, arc, points[:, 2]),
    ]).T

def arc_length(points):
    return np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1))

def norm(v):
    n = np.linalg.norm(v)
    if n < EPS:
        raise RuntimeError("Zero-length vector.")
    return v / n

def make_fiducial_list(name, points):
    node = slicer.mrmlScene.AddNewNodeByClass("vtkMRMLMarkupsFiducialNode", name)
    node.CreateDefaultDisplayNodes()
    for p in points:
        node.AddControlPoint(vtk.vtkVector3d(float(p[0]), float(p[1]), float(p[2])))
    return node

# ==================================================
# STEP 1: READ POINTS
# ==================================================

needle_pts = get_pts(needleNode)  # already base -> tip
cat_pts    = get_pts(catNode)

# ==================================================
# STEP 2: REORIENT CATHETER TO BASE -> TIP
# ==================================================

if CATheter_FIRST_POINT_IS_BASE:
    cat_reoriented = cat_pts.copy()
else:
    cat_reoriented = cat_pts[::-1].copy()

cat_reoriented_node = make_fiducial_list("catheter_1_reoriented", cat_reoriented)

# ==================================================
# STEP 3: RESAMPLE CATHETER ONLY
# ==================================================

cat_resampled = resample(cat_reoriented, DS)

# ==================================================
# STEP 4: COMPUTE LENGTHS
# ==================================================

L_needle = arc_length(needle_pts)
L_cat    = arc_length(cat_resampled)
L_ext    = L_needle - L_cat

if L_ext < -EPS:
    raise RuntimeError("Catheter already longer than needle.")

n_ext = int(round(max(L_ext, 0.0) / DS))

# ==================================================
# STEP 5: APPEND STRAIGHT EXTENSION TO CATHETER END
# ==================================================
# Since catheter is now base -> tip, the base side is the FIRST point.
# The extension goes toward the base-side continuation, i.e. BEFORE the first point.

if n_ext > 0:
    base = cat_resampled[0]
    direction = norm(cat_resampled[0] - cat_resampled[1])  # outward from base

    ext_pts = np.array([
        base + direction * ((n_ext - i) * DS) for i in range(n_ext)
    ], dtype=float)

    cat_extended = np.vstack([ext_pts, cat_resampled])
else:
    cat_extended = cat_resampled.copy()

cat_extended_node = make_fiducial_list("catheter_1_extended", cat_extended)

# ==================================================
# STEP 6: ALIGN BASE-SIDE STRAIGHT SEGMENTS
# ==================================================
# Needle is base -> tip, so use first N points.
# Catheter is now base -> tip, so use first N points.

n_align = min(len(needle_pts), len(cat_extended))
if n_align < 3:
    raise RuntimeError("Not enough alignment points for rigid transform.")

needle_align = needle_pts[:n_align]
cat_align    = cat_extended[:n_align]

# ==================================================
# STEP 7: RIGID ALIGNMENT
# ==================================================

source = vtk.vtkPoints()
target = vtk.vtkPoints()

for i in range(n_align):
    source.InsertNextPoint(float(needle_align[i, 0]), float(needle_align[i, 1]), float(needle_align[i, 2]))
    target.InsertNextPoint(float(cat_align[i, 0]), float(cat_align[i, 1]), float(cat_align[i, 2]))

t = vtk.vtkLandmarkTransform()
t.SetSourceLandmarks(source)
t.SetTargetLandmarks(target)
t.SetModeToRigidBody()
t.Update()

out = slicer.mrmlScene.AddNewNodeByClass(
    "vtkMRMLLinearTransformNode",
    "Needle_to_Catheter"
)
out.SetMatrixTransformToParent(t.GetMatrix())
needleNode.SetAndObserveTransformNodeID(out.GetID())

# ==================================================
# STEP 8: RMSE
# ==================================================

rmse = 0.0
for i in range(n_align):
    p = np.array(t.TransformPoint(
        float(needle_align[i, 0]),
        float(needle_align[i, 1]),
        float(needle_align[i, 2])
    ))
    q = cat_align[i]
    rmse += np.sum((p - q) ** 2)

rmse = np.sqrt(rmse / n_align)

print("----------------------------------")
print("catheter_1 reoriented to base -> tip")
print("Needle points:", len(needle_pts))
print("Catheter resampled points:", len(cat_resampled))
print("Extension points:", n_ext)
print("Alignment points:", n_align)
print("RMSE:", rmse)
print("Reoriented node:", cat_reoriented_node.GetName())
print("Extended node:", cat_extended_node.GetName())
print("----------------------------------")
