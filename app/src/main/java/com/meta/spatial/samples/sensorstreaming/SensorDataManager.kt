package com.meta.spatial.samples.sensorstreaming

import android.util.Log
import com.meta.spatial.core.Vector3
import com.meta.spatial.core.Quaternion
import com.meta.spatial.runtime.ControllerPose
import com.meta.spatial.runtime.JointPose

/**
 * Data model for an individual joint's tracking data.
 * Captured from the Meta Spatial SDK / OpenXR tracking buffers.
 */
data class JointData(
    val id: Int,
    val jointName: String,
    val pos: List<Float>, // [x, y, z] in meters
    val rot: List<Float>  // [x, y, z, w] quaternion
)

/**
 * Data model for headset/head positional tracking.
 */
data class HeadPoseData(
    val pos: List<Float>, // [x, y, z]
    val rot: List<Float>  // [x, y, z, w]
)

/**
 * Data model for gaze tracking.
 * If eye tracking is unavailable, this approximates using head direction.
 */
data class GazeData(
    val origin: List<Float>,    // [x, y, z]
    val direction: List<Float> // [x, y, z] unit vector
)

/**
 * Data model for depth approximation.
 * Since raw depth maps are not available in the Meta Spatial SDK, 
 * we approximate 'depth' as the Euclidean distance between key tracked joints.
 * This provides a measure of how far objects (hands) are from the user's perspective (head).
 */
data class DepthEstimateData(
    val headToLeftHand: Float,
    val headToRightHand: Float,
    val handToHand: Float
)

/**
 * Unified data model for a single frame of all sensor data.
 * Includes performance metrics and all tracked sensor streams.
 */
data class FrameData(
    val timestamp: Long,
    val fps: Float,
    val frameTimeMs: Long,
    val memoryMb: Long,
    val joints: List<JointData>,
    val headPose: HeadPoseData,
    val gaze: GazeData,
    val depthEstimate: DepthEstimateData
)

/**
 * SensorDataManager: Responsible for processing raw tracking buffers into structured data.
 * 
 * Architecture Role:
 * - Decouples tracking data processing from the Activity/Rendering logic.
 * - Handles filtering of noisy/invalid joint data.
 * - Aggregates multiple sensor streams (Body, Head, Gaze).
 * - Manages performance metrics (FPS, Memory).
 * 
 * Data Flow (Per Frame):
 * 1. Fetch tracking buffers from SDK.
 * 2. Validate joint tracking quality.
 * 3. Extract head/gaze data.
 * 4. Compute performance metrics.
 * 5. Transform into unified JSON and log.
 */
@OptIn(com.meta.spatial.core.SpatialSDKExperimentalAPI::class)
class SensorDataManager {

    // Performance tracking state
    private var lastFrameTime: Long = 0
    private var currentFps: Float = 0f
    private var lastFpsUpdate: Long = 0
    private var frameCount: Int = 0

    /**
     * Mapping from Joint ID to human-readable names.
     * This mapping follows the standard XR_META_body_tracking_full_body joint convention.
     */
    private val JOINT_NAME_MAP = mapOf(
        0 to "ROOT", 1 to "HIPS", 2 to "SPINE_LOWER", 3 to "SPINE_MIDDLE", 4 to "SPINE_UPPER",
        5 to "CHEST", 6 to "NECK", 7 to "HEAD", 8 to "LEFT_SHOULDER", 9 to "LEFT_SCAPULA",
        10 to "LEFT_ARM_UPPER", 11 to "LEFT_ARM_LOWER", 12 to "LEFT_ELBOW", 13 to "LEFT_WRIST",
        14 to "RIGHT_SHOULDER", 15 to "RIGHT_SCAPULA", 16 to "RIGHT_ARM_UPPER", 17 to "RIGHT_ARM_LOWER",
        18 to "RIGHT_ELBOW", 19 to "RIGHT_WRIST", 20 to "LEFT_HAND_PALM", 44 to "RIGHT_HAND_PALM",
        70 to "LEFT_LEG_UPPER", 71 to "LEFT_LEG_LOWER", 73 to "LEFT_FOOT_ANKLE",
        77 to "RIGHT_LEG_UPPER", 78 to "RIGHT_LEG_LOWER", 80 to "RIGHT_FOOT_ANKLE"
    )

    fun getJointName(id: Int): String = JOINT_NAME_MAP[id] ?: "JOINT_$id"

    /**
     * Placeholder for future Depth API integration.
     * Currently not available in the base Spatial SDK body tracking configuration.
     */
    fun getDepthData(): String {
        // Depth data not available in current SDK — placeholder implemented
        return "DEPTH_NOT_SUPPORTED"
    }

    /**
     * Aggregates all sensor data into a structured FrameData object.
     */
    fun assembleFrame(
        jointPoses: List<ControllerPose>,
        headPos: Vector3,
        headRot: Quaternion,
        gazeOrigin: Vector3,
        gazeDir: Vector3
    ): FrameData {
        val currentTime = System.currentTimeMillis()
        val deltaTime = if (lastFrameTime > 0) currentTime - lastFrameTime else 0
        
        updateFps(currentTime)
        val memoryMb = (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024 * 1024)

        val trackedJoints = jointPoses.mapIndexedNotNull { index, pose ->
            if (isPoseTracked(pose)) {
                JointData(
                    id = index,
                    jointName = getJointName(index),
                    pos = listOf(pose.pose.t.x, pose.pose.t.y, pose.pose.t.z),
                    rot = listOf(pose.pose.q.x, pose.pose.q.y, pose.pose.q.z, pose.pose.q.w)
                )
            } else null
        }

        // --- DEPTH APPROXIMATION LOGIC ---
        // Raw depth maps are unavailable. We compute Euclidean distance between 
        // the head (ID 7) and palm joints (IDs 20, 44) to estimate depth interaction.
        val headPoseObj = jointPoses.getOrNull(7)?.pose?.t
        val leftHandPoseObj = jointPoses.getOrNull(20)?.pose?.t
        val rightHandPoseObj = jointPoses.getOrNull(44)?.pose?.t

        val depthEstimate = DepthEstimateData(
            headToLeftHand = if (headPoseObj != null && leftHandPoseObj != null) (headPoseObj - leftHandPoseObj).length() else 0f,
            headToRightHand = if (headPoseObj != null && rightHandPoseObj != null) (headPoseObj - rightHandPoseObj).length() else 0f,
            handToHand = if (leftHandPoseObj != null && rightHandPoseObj != null) (leftHandPoseObj - rightHandPoseObj).length() else 0f
        )

        lastFrameTime = currentTime

        return FrameData(
            timestamp = currentTime,
            fps = currentFps,
            frameTimeMs = deltaTime,
            memoryMb = memoryMb,
            joints = trackedJoints,
            headPose = HeadPoseData(listOf(headPos.x, headPos.y, headPos.z), listOf(headRot.x, headRot.y, headRot.z, headRot.w)),
            gaze = GazeData(listOf(gazeOrigin.x, gazeOrigin.y, gazeOrigin.z), listOf(gazeDir.x, gazeDir.y, gazeDir.z)),
            depthEstimate = depthEstimate
        )
    }

    private fun isPoseTracked(pose: ControllerPose): Boolean {
        val requiredBits = JointPose.LocationValidBit or JointPose.LocationTrackedBit or 
                          JointPose.OrientationValidBit or JointPose.OrientationTrackedBit
        return (pose.flags and requiredBits) == requiredBits
    }

    private fun updateFps(currentTime: Long) {
        frameCount++
        if (currentTime - lastFpsUpdate >= 1000) {
            currentFps = (frameCount * 1000f) / (currentTime - lastFpsUpdate)
            frameCount = 0
            lastFpsUpdate = currentTime
            
            // Log performance stats occasionally
            Log.d("SENSOR_PERF", "FPS: ${"%.1f".format(currentFps)} | Memory: ${ (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory()) / (1024 * 1024) }MB")
        }
    }

    /**
     * Unified JSON streaming under the "SENSOR_STREAM" tag.
     */
    fun streamFrameData(frame: FrameData) {
        val json = """{
            "timestamp": ${frame.timestamp},
            "fps": ${"%.1f".format(frame.fps)},
            "memory_mb": ${frame.memoryMb},
            "head_pose": {"pos": ${frame.headPose.pos}, "rot": ${frame.headPose.rot}},
            "gaze": {"origin": ${frame.gaze.origin}, "direction": ${frame.gaze.direction}},
            "depth_estimate": {
                "head_to_left_hand": ${"%.4f".format(frame.depthEstimate.headToLeftHand)},
                "head_to_right_hand": ${"%.4f".format(frame.depthEstimate.headToRightHand)},
                "hand_to_hand": ${"%.4f".format(frame.depthEstimate.handToHand)}
            },
            "joints": [${frame.joints.joinToString(",") { formatJointJson(it) }}]
        }""".trimIndent().replace("\n", "").replace("  ", "")
        
        Log.d("SENSOR_STREAM", json)
    }

    private fun formatJointJson(joint: JointData): String {
        return """{"id":${joint.id},"name":"${joint.jointName}","pos":[${"%.4f".format(joint.pos[0])},${"%.4f".format(joint.pos[1])},${"%.4f".format(joint.pos[2])}],"rot":[${"%.4f".format(joint.rot[0])},${"%.4f".format(joint.rot[1])},${"%.4f".format(joint.rot[2])},${"%.4f".format(joint.rot[3])}]}"""
    }
}
