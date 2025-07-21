# kcl_project/utils/spawn.py

from pxr import Usd, UsdGeom, Sdf, Gf

# kcl_project/utils/spawn.py

def spawn_usdz_asset(stage, usdz_path, prim_path, position=(0,0,0), scale=(1,1,1), rotation=(0,0,0), orientation=None):
    """
    Spawn a USDZ asset with position, scale, and rotation/orientation.
    
    Args:
        stage: USD stage
        usdz_path: Path to the USDZ file
        prim_path: USD prim path where to spawn the asset
        position: (x, y, z) position tuple
        scale: (x, y, z) scale tuple
        rotation: (x, y, z) Euler angles in degrees (used if orientation is None)
        orientation: (w, x, y, z) quaternion (takes priority over rotation if provided)
    """
    # Remove existing prim at the path if it exists
    prim = stage.GetPrimAtPath(prim_path)
    if prim and prim.IsValid():
        stage.RemovePrim(prim_path)
    
    # Create and set up the new prim
    from pxr import UsdGeom
    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.AddTranslateOp().Set(position)
    xform.AddScaleOp().Set(scale)
    
    # Use orientation (quaternion) if provided, otherwise use rotation (Euler)
    if orientation is not None:
        # Use quaternion orientation
        quat = Gf.Quatf(float(orientation[0]), float(orientation[1]), float(orientation[2]), float(orientation[3]))
        xform.AddOrientOp().Set(quat)
    else:
        # Use Euler rotation
        xform.AddRotateXYZOp().Set(rotation)
    
    xform.GetPrim().GetReferences().AddReference(usdz_path)

def clear_scene(stage, prim_paths):
    for path in prim_paths:
        prim = stage.GetPrimAtPath(path)
        if prim and prim.IsValid():
            stage.RemovePrim(path)

