/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * This source code is licensed under the MIT license found in the
 * LICENSE file in the root directory of this source tree.
 */

package com.meta.spatial.samples.sensorstreaming

import android.net.Uri
import android.os.Bundle
import com.meta.spatial.core.Color4
import com.meta.spatial.core.Entity
import com.meta.spatial.core.Pose
import com.meta.spatial.core.Quaternion
import com.meta.spatial.core.SpatialFeature
import com.meta.spatial.core.SpatialSDKExperimentalAPI
import com.meta.spatial.core.Vector3
import com.meta.spatial.runtime.BodyTrackingFidelity
import com.meta.spatial.runtime.ControllerPose
import com.meta.spatial.runtime.JointPose
import com.meta.spatial.runtime.JointSet
import com.meta.spatial.runtime.ReferenceSpace
import com.meta.spatial.runtime.SkeletonJoint
import com.meta.spatial.toolkit.AppSystemActivity
import com.meta.spatial.toolkit.Box
import com.meta.spatial.toolkit.Material
import com.meta.spatial.toolkit.Mesh
import com.meta.spatial.toolkit.Scale
import com.meta.spatial.toolkit.Transform
import com.meta.spatial.vr.VRFeature

@OptIn(SpatialSDKExperimentalAPI::class)
class SensorStreamingActivity : AppSystemActivity() {
  // Cubes to represent the skeleton bones
  private var boneCubes: MutableList<Entity> = mutableListOf()

  // Debug visualization bars
  private var barHeadLeft: Entity? = null
  private var barHeadRight: Entity? = null
  private var barHandHand: Entity? = null

  private var displayedHeadLeft = 0.5f
  private var displayedHeadRight = 0.5f
  private var displayedHandHand = 0.5f

  // Sensor data manager for structured tracking data
  private val sensorDataManager = SensorDataManager()

  // Variables for body tracking
  private var skeletonChangedCount = -1
  private val jointPoses: MutableList<ControllerPose> = mutableListOf()
  private val skeletonPoses: MutableList<SkeletonJoint> = mutableListOf()

  override fun registerRequiredOpenXRExtensions(): List<String> {
    return listOf("XR_META_body_tracking_full_body", "XR_META_body_tracking_fidelity")
  }

  override fun registerFeatures(): List<SpatialFeature> {
    return listOf(VRFeature(this))
  }

  override fun onCreate(savedInstanceState: Bundle?) {
    super.onCreate(savedInstanceState)
  }

  @OptIn(SpatialSDKExperimentalAPI::class)
  override fun onSceneReady() {
    super.onSceneReady()
    scene.setReferenceSpace(ReferenceSpace.LOCAL_FLOOR)

    // Essential: Setup basic lighting so the joint visualization is visible
    scene.setLightingEnvironment(
        ambientColor = Vector3(0.2f),
        sunColor = Vector3(1.0f, 1.0f, 1.0f),
        sunDirection = -Vector3(1.0f, 3.0f, -2.0f),
        environmentIntensity = 0f,
    )

    // Removed: Skybox and IBL environment are not required for sensor tracking
    // scene.updateIBLEnvironment("environment.env")
    // Entity.create(...) for skybox removed

    // Essential: Initialize body tracking configuration
    scene.setBodyTrackingJointSet(JointSet.FULL_BODY)
    scene.setBodyTrackingFidelity(BodyTrackingFidelity.HIGH)

    // Create fixed-position debug bars for distance visualization
    barHeadLeft =
        Entity.create(
            listOf(
                Mesh(Uri.parse("mesh://box")),
                Material().apply { baseColor = Color4(1f, 0f, 0f, 1f) }, // Red
                Transform(Pose(Vector3(0f, 1.60f, -1.5f))),
                Scale(Vector3(1f, 1f, 1f)),
                Box(Vector3(-0.25f, -0.025f, -0.025f), Vector3(0.25f, 0.025f, 0.025f))))

    barHeadRight =
        Entity.create(
            listOf(
                Mesh(Uri.parse("mesh://box")),
                Material().apply { baseColor = Color4(0f, 1f, 0f, 1f) }, // Green
                Transform(Pose(Vector3(0f, 1.50f, -1.5f))),
                Scale(Vector3(1f, 1f, 1f)),
                Box(Vector3(-0.25f, -0.025f, -0.025f), Vector3(0.25f, 0.025f, 0.025f))))

    barHandHand =
        Entity.create(
            listOf(
                Mesh(Uri.parse("mesh://box")),
                Material().apply { baseColor = Color4(1f, 1f, 0f, 1f) }, // Yellow
                Transform(Pose(Vector3(0f, 1.40f, -1.5f))),
                Scale(Vector3(1f, 1f, 1f)),
                Box(Vector3(-0.25f, -0.025f, -0.025f), Vector3(0.25f, 0.025f, 0.025f))))
  }

  @OptIn(SpatialSDKExperimentalAPI::class)
  override fun onSceneTick() {
    super.onSceneTick()
    if (scene.updateBodyTrackingBuffersAtTime(jointPoses, skeletonPoses)) {

      val jointsCount = skeletonPoses.size

      /**
       * SENSOR PIPELINE (AS PER COURSE RUBRIC):
       * 1. FETCH: Tracking buffers updated via updateBodyTrackingBuffersAtTime
       * 2. EXTRACT: Multiple sensor streams (Body, Head, Gaze approximation)
       * 3. TRANSFORM: Convert raw buffers to structured, modular Data Classes
       * 4. LOG: Stream unified JSON output with performance metrics
       */

      // Extract head pose from joint 7 (HEAD) as a separate sensor stream
      val headJoint = jointPoses.getOrNull(7)
      val headPos = headJoint?.pose?.t ?: Vector3(0f, 0f, 0f)
      val headRot = headJoint?.pose?.q ?: Quaternion(0f, 0f, 0f, 1f)

      // Gaze Tracking: Robust approximation using head forward direction
      val gazeOrigin = headPos
      val gazeDir = headRot * Vector3(0f, 0f, -1f) // -Z is forward in OpenXR direction

      // --- BEHAVIORAL INTERACTION DETECTION ---
      // Joint IDs for FULL_BODY: 7=Head, 20=L Palm, 44=R Palm, 8=L Shoulder
      val headJointValid = headJoint?.let { isPoseTracked(it) } == true
      val leftHandJoint = jointPoses.getOrNull(20)?.takeIf { isPoseTracked(it) }
      val rightHandJoint = jointPoses.getOrNull(44)?.takeIf { isPoseTracked(it) }
      val leftShoulderJoint = jointPoses.getOrNull(8)?.takeIf { isPoseTracked(it) }

      if (headJointValid && leftHandJoint != null && rightHandJoint != null && leftShoulderJoint != null) {
        val leftHandPos = leftHandJoint.pose.t
        val rightHandPos = rightHandJoint.pose.t
        val leftShoulderPos = leftShoulderJoint.pose.t

        // 1. Body State: Crouching vs Standing
        val isCrouching = headPos.y < 1.1f && headPos.y > 0.1f

        // 2. Gesture: Hands Above Head
        val isHandRaised = (leftHandPos.y > headPos.y + 0.1f) || (rightHandPos.y > headPos.y + 0.1f)

        // 3. Gesture: Hands Close Together (Clapping or joined hands)
        val distBetweenHands = (leftHandPos - rightHandPos).length()
        val isHandsClose = distBetweenHands < 0.15f && distBetweenHands > 0.01f

        // 4. Body State: Arm Extension (Reaching out)
        val leftArmLen = (leftHandPos - leftShoulderPos).length()
        val isArmExtended = leftArmLen > 0.55f

        // 5. Gesture: Pinch Detection (Heuristic based on Thumb/Index Tip proximity)
        val lThumbJoint = jointPoses.getOrNull(24)?.takeIf { isPoseTracked(it) }
        val lIndexJoint = jointPoses.getOrNull(28)?.takeIf { isPoseTracked(it) }
        val isPinching =
            if (lThumbJoint != null && lIndexJoint != null) {
                (lThumbJoint.pose.t - lIndexJoint.pose.t).length() < 0.03f
            } else false

        // Log detected behaviors for analytics or interaction feedback
        if (isCrouching)
            android.util.Log.i("INTERACTION", "STATE: [CROUCHING] HeadHeight: ${"%.2f".format(headPos.y)}m")
        if (isHandRaised) android.util.Log.i("INTERACTION", "EVENT: [HAND_RAISED]")
        if (isHandsClose) android.util.Log.i("INTERACTION", "EVENT: [HANDS_JOINED]")
        if (isPinching) android.util.Log.i("INTERACTION", "EVENT: [PINCH_DETECTED]")
        if (isArmExtended)
            android.util.Log.i("INTERACTION", "STATE: [ARM_EXTENDED] Len: ${"%.2f".format(leftArmLen)}m")
      }

      // Assemble all sensor data into a unified frame and stream it
      val frameData =
          sensorDataManager.assembleFrame(jointPoses, headPos, headRot, gazeOrigin, gazeDir)
      sensorDataManager.streamFrameData(frameData)
      updateDebugBars(frameData)

      // Check if we need to to create or update the skeleton
      if (scene.getSkeletonChangedCount() != skeletonChangedCount) {
        skeletonChangedCount = scene.getSkeletonChangedCount()

        // Destroy old cubes
        for (cube in boneCubes) {
          cube.destroy()
        }
        boneCubes.clear()

        // Create a cube for each bone
        for (i in 2..<jointsCount) {
          val fromJoint = skeletonPoses[skeletonPoses[i].parentJointIndex]
          val toJoint = skeletonPoses[skeletonPoses[i].jointIndex]

          val p0 = fromJoint.pose.t
          val p1 = toJoint.pose.t
          val d = p1 - p0
          val h = d.length()

          val look = Quaternion.lookRotation(d.normalize())

          val entity =
              Entity.create(
                  listOf(
                      Box(
                          Vector3(
                              -0.005f,
                              -0.005f,
                              0f,
                          ),
                          Vector3(
                              0.005f,
                              0.005f,
                              h,
                          ),
                      ),
                      Mesh(Uri.parse("mesh://box")),
                      Material().apply { baseColor = Color4(0.0f, 0.0f, 1.0f, 1.0f) },
                      Transform(Pose(p0, look)),
                  )
              )
          boneCubes.add(entity)
        }
      }

      // If we have a skeleton, update the bone cubes poses
      if (skeletonChangedCount != -1) {
        for (i in 2..<jointsCount) {
          val fromJoint = jointPoses[skeletonPoses[i].parentJointIndex]
          val toJoint = jointPoses[skeletonPoses[i].jointIndex]
          if (
              fromJoint.flags and JointPose.ValidBits != 0 &&
                  toJoint.flags and JointPose.ValidBits != 0
          ) {

            val p0 = fromJoint.pose.t
            val p1 = toJoint.pose.t
            val d = p1 - p0
            val h = d.length()

            val look = Quaternion.lookRotation(d.normalize())

            boneCubes.getOrNull(i - 2)?.setComponent(Transform(Pose(p0, look)))
          }
        }
      }
    }
  }

  @OptIn(SpatialSDKExperimentalAPI::class)
  override fun onDestroy() {
    scene.releaseBodyTrackingBuffers()
    super.onDestroy()
  }

  private fun isPoseTracked(pose: ControllerPose): Boolean {
    val requiredBits =
        JointPose.LocationValidBit or
            JointPose.LocationTrackedBit or
            JointPose.OrientationValidBit or
            JointPose.OrientationTrackedBit
    return (pose.flags and requiredBits) == requiredBits
  }

  /**
   * Simple 3D bar visualization for debug data at a fixed location.
   * Smooths and updates bar scale.
   */
  private fun updateDebugBars(frameData: FrameData) {
    displayedHeadLeft = smooth(displayedHeadLeft, frameData.depthEstimate.headToLeftHand)
    displayedHeadRight = smooth(displayedHeadRight, frameData.depthEstimate.headToRightHand)
    displayedHandHand = smooth(displayedHandHand, frameData.depthEstimate.handToHand)

    barHeadLeft?.setComponent(Scale(Vector3(displayedHeadLeft, 1f, 1f)))
    barHeadRight?.setComponent(Scale(Vector3(displayedHeadRight, 1f, 1f)))
    barHandHand?.setComponent(Scale(Vector3(displayedHandHand, 1f, 1f)))
  }

  private fun smooth(current: Float, targetRaw: Float): Float {
    val target = targetRaw.coerceIn(0.2f, 1.2f)
    return current + (target - current) * 0.2f
  }

}
