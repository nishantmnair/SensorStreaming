# Meta Quest Sensor Streaming App

## Overview
Android/Meta Quest Pro app that streams body tracking, head pose, gaze approximation, depth approximation, and performance metrics. Built using the Meta Spatial SDK.

## Sensor Streams
- **Body joints**: Full skeletal tracking data.
- **Head pose**: Position and orientation of the headset.
- **Gaze/head-forward direction**: Approximation of gaze using head forward vector.
- **Depth estimate**: Euclidean distance between head-to-hand and hand-to-hand joints.
- **Performance**: Real-time FPS and memory usage metrics.

## Depth Limitation
Raw camera access and raw depth maps are not exposed through the standard Spatial SDK. Depth is approximated using Euclidean distance between tracked joints (Head to Palms) to provide interaction context.

## How to Run
1. Open the project in Android Studio.
2. Connect your Meta Quest (Quest 3/Pro recommended) in developer mode.
3. Build and install the app.
4. View the live sensor data in Logcat using the filter `tag:SENSOR_STREAM`.
5. Performance stats are logged under `tag:SENSOR_PERF`.

## Performance Profiling
The app logs FPS and memory usage during runtime. Additional CPU, memory, and network profiling can be captured using Android Studio Profiler while the app runs on the headset.
