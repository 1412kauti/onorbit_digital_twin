#!/usr/bin/env python3
"""
Test script for simulation view error recovery.

This script can be run from Isaac Sim's script editor to test
the simulation view recovery functionality.
"""

import carb
import omni.timeline
from kcl_project.utils.readings import recover_from_simulation_view_error, get_simulation_diagnostics

def test_recovery():
    """Test the simulation view recovery function."""
    
    carb.log_info("[Test] Starting simulation view recovery test...")
    
    # Get current diagnostics
    diagnostics = get_simulation_diagnostics()
    carb.log_info(f"[Test] Current diagnostics: {diagnostics}")
    
    # Test recovery function
    success = recover_from_simulation_view_error()
    
    if success:
        carb.log_info("[Test] Recovery function completed successfully!")
        
        # Get diagnostics after recovery
        post_diagnostics = get_simulation_diagnostics()
        carb.log_info(f"[Test] Post-recovery diagnostics: {post_diagnostics}")
        
        return True
    else:
        carb.log_error("[Test] Recovery function failed!")
        return False

def test_timeline_control():
    """Test timeline control functionality."""
    
    carb.log_info("[Test] Starting timeline control test...")
    
    try:
        timeline = omni.timeline.get_timeline_interface()
        if not timeline:
            carb.log_error("[Test] Timeline interface not available")
            return False
        
        # Test stopping and starting
        was_playing = timeline.is_playing()
        carb.log_info(f"[Test] Timeline was playing: {was_playing}")
        
        if was_playing:
            carb.log_info("[Test] Stopping timeline...")
            timeline.stop()
            
            import time
            time.sleep(1.0)
            
            carb.log_info("[Test] Starting timeline...")
            timeline.play()
            
            time.sleep(1.0)
            
        carb.log_info("[Test] Timeline control test completed successfully!")
        return True
        
    except Exception as e:
        carb.log_error(f"[Test] Timeline control test failed: {str(e)}")
        return False

if __name__ == "__main__":
    carb.log_info("[Test] Running simulation recovery tests...")
    
    # Run tests
    recovery_success = test_recovery()
    timeline_success = test_timeline_control()
    
    if recovery_success and timeline_success:
        carb.log_info("[Test] All tests passed! Simulation recovery should work correctly.")
    else:
        carb.log_warn("[Test] Some tests failed. Check the logs for details.")
