# kcl_project/utils/camera.py

from pxr import UsdGeom, Sdf

def set_camera_lock(stage, camera_prim_path, locked=True):
    """
    Sets the camera lock state for an existing camera prim.
    - camera_prim_path: Path to the camera prim (e.g., "/World/Cameras/StageCam1")
    - locked: True to lock, False to unlock
    """
    prim = stage.GetPrimAtPath(camera_prim_path)
    if prim and prim.IsValid():
        # Use string directly instead of Tf.Token
        prim.CreateAttribute("omni:kit:cameraLock", Sdf.ValueTypeNames.Bool).Set(locked)
        return True
    return False

def create_camera(stage, prim_path, position, rotation, 
                  focal_length=50.0, horizontal_aperture=20.955, vertical_aperture=15.2908, 
                  focus_distance=10.0, locked=False):
    """
    Adds a USD camera prim with optional camera lock.
    """
    camera = UsdGeom.Camera.Define(stage, prim_path)
    xform = UsdGeom.Xformable(camera)
    xform.ClearXformOpOrder()
    xform.AddRotateXYZOp().Set(rotation)
    xform.AddTranslateOp().Set(position)
    
    # Camera properties
    camera.CreateFocalLengthAttr().Set(focal_length)
    camera.CreateHorizontalApertureAttr().Set(horizontal_aperture)
    camera.CreateVerticalApertureAttr().Set(vertical_aperture)
    camera.CreateFocusDistanceAttr().Set(focus_distance)
    camera.CreateProjectionAttr().Set('perspective')
    
    # Set camera lock if requested
    if locked:
        prim = camera.GetPrim()
        # Use string directly instead of Tf.Token
        prim.CreateAttribute("omni:kit:cameraLock", Sdf.ValueTypeNames.Bool).Set(True)
    
    return camera
