# Simulation View Invalidation Fix

## Problem Description

You were experiencing recurring errors in Isaac Sim 4.5 with messages like:
```
[Error] [omni.physx.tensors.plugin] Simulation view object is invalidated and cannot be used again to call getVelocities
```

These errors occur when:
1. Physics simulation views become invalidated during timeline operations
2. Sensor readings are attempted on invalidated physics objects
3. Race conditions exist between simulation updates and sensor data access
4. Timeline is stopped/started while sensors are active

## Root Cause

Isaac Sim 4.5 uses PhysX simulation views for efficient sensor data access. When the timeline is stopped, restarted, or when scene modifications occur during simulation, these views become invalidated but aren't automatically recreated, causing the `getVelocities` errors.

## Solution Implemented

### 1. Enhanced Recovery Function (`extension.py`)

Updated `recover_from_simulation_error()` method with:
- **Graceful timeline management**: Properly stop/start timeline with adequate wait times
- **PhysX view clearing**: Force recreation of physics simulation contexts
- **Garbage collection**: Clear cached references that might hold invalidated views
- **Extended stabilization times**: Allow system to fully reset before restart

### 2. Advanced Recovery Module (`readings.py`)

Added `recover_from_simulation_view_error()` function that:
- Stops timeline gracefully with extended cleanup time (1.5s)
- Forces PhysX interface to reload physics from USD
- Performs garbage collection to clear stale references  
- Waits for system stabilization (2.0s)
- Restarts timeline with proper initialization time (1.5s)

### 3. Improved Error Handling

Enhanced error handling in sensor reading functions:
- Detect simulation view invalidation errors specifically
- Attempt automatic recovery when errors are detected
- Skip problematic sensors gracefully without crashing the entire system
- Provide detailed logging for troubleshooting

### 4. Preventive Measures

- **Longer wait times**: Extended delays during timeline operations to prevent race conditions
- **Proper cleanup**: Force PhysX to reload physics contexts
- **Memory management**: Garbage collection to clear references
- **Graceful degradation**: Continue operation even if some sensors fail

## Usage

The fixes are automatically applied when simulation view errors occur. No manual intervention required.

### Manual Recovery

If you need to manually trigger recovery:

```python
from kcl_project.extension import KclProjectExtension
extension_instance.recover_from_simulation_error()
```

Or from the readings module:

```python
from kcl_project.utils.readings import recover_from_simulation_view_error
success = recover_from_simulation_view_error()
```

### Testing

Run the provided test script to verify the fix:

```python
exec(open('/home/kaito/.local/share/ov/data/Kit/Isaac-Sim Full/4.5/exts/3/kcl_project/test_simulation_recovery.py').read())
```

## Key Improvements

1. **Automatic Recovery**: Errors are detected and recovery is attempted automatically
2. **Better Timing**: Extended wait times prevent race conditions
3. **Robust Cleanup**: Forces PhysX to recreate invalidated views
4. **Graceful Handling**: System continues working even if individual sensors fail
5. **Detailed Logging**: Better debugging information for future issues

## Expected Behavior

After implementing these fixes:
- ✅ Simulation view errors should occur much less frequently
- ✅ When they do occur, automatic recovery should resolve them
- ✅ IMU sensor data should continue to update reliably  
- ✅ Timeline operations should be more stable
- ✅ Better error messages for troubleshooting

## Monitoring

The system now provides better diagnostics:

```python
from kcl_project.utils.readings import get_simulation_diagnostics
print(get_simulation_diagnostics())
```

This will show timeline status, physics scene status, and recommendations for fixing any issues.

## Notes

- The fix adds some latency during recovery (3-4 seconds total) but ensures system stability
- Recovery attempts are logged with detailed information
- If automatic recovery fails repeatedly, manual scene reload may be required
- Compatible with Isaac Sim 4.5 and the omni.physx.tensors plugin
