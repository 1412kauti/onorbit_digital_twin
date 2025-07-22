# kcl_project/utils/background.py

import os
from pxr import UsdLux, UsdGeom

def set_background(stage, hdri_path, dome_path="/World/Environment", intensity=5000.0, exposure=0.0):
    # Remove existing DomeLight at the specified path
    if stage.GetPrimAtPath(dome_path):
        stage.RemovePrim(dome_path)
    # Create DomeLight
    dome = UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateTextureFileAttr().Set(hdri_path)
    dome.CreateIntensityAttr().Set(intensity)
    dome.CreateExposureAttr().Set(exposure)
    UsdGeom.Imageable(dome.GetPrim()).MakeVisible()
