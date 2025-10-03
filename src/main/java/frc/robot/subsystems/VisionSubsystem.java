// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;

/**
 * Vision subsystem for AprilTag-based robot localization
 * Uses dual PhotonVision cameras for 360-degree coverage
 */
public class VisionSubsystem extends SubsystemBase {
  // Front camera
  private final PhotonCamera m_frontCamera;
  private final PhotonPoseEstimator m_frontPoseEstimator;

  // Rear camera
  private final PhotonCamera m_rearCamera;
  private final PhotonPoseEstimator m_rearPoseEstimator;

  private final CommandSwerveDrivetrain m_drivetrain;

  private boolean m_frontInitialized = false;
  private boolean m_rearInitialized = false;
  private double m_lastFrontEstimateTimestamp = 0.0;
  private double m_lastRearEstimateTimestamp = 0.0;

  /**
   * Creates a new VisionSubsystem with dual cameras
   * @param drivetrain The swerve drivetrain for pose updates
   */
  public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;

    // Initialize front camera
    PhotonCamera frontCamera = null;
    PhotonPoseEstimator frontEstimator = null;
    try {
      frontCamera = new PhotonCamera(VisionConstants.FRONT_CAMERA_NAME);

      AprilTagFieldLayout fieldLayout = FieldConfiguration.getFieldLayout(VisionConstants.FIELD_MODE);

      Transform3d robotToFrontCamera = new Transform3d(
          new Translation3d(
              VisionConstants.FRONT_CAMERA_X_OFFSET,
              VisionConstants.FRONT_CAMERA_Y_OFFSET,
              VisionConstants.FRONT_CAMERA_Z_OFFSET
          ),
          new Rotation3d(
              Math.toRadians(VisionConstants.FRONT_CAMERA_ROLL_DEGREES),
              Math.toRadians(VisionConstants.FRONT_CAMERA_PITCH_DEGREES),
              Math.toRadians(VisionConstants.FRONT_CAMERA_YAW_DEGREES)
          )
      );

      frontEstimator = new PhotonPoseEstimator(
          fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToFrontCamera
      );
      frontEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      m_frontInitialized = true;
      DataLogManager.log("Front camera initialized: " + VisionConstants.FRONT_CAMERA_NAME);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize front camera: " + e.getMessage(), true);
      DataLogManager.log("ERROR: Front camera initialization failed - " + e.getMessage());
      m_frontInitialized = false;
    }
    m_frontCamera = frontCamera;
    m_frontPoseEstimator = frontEstimator;

    // Initialize rear camera
    PhotonCamera rearCamera = null;
    PhotonPoseEstimator rearEstimator = null;
    try {
      rearCamera = new PhotonCamera(VisionConstants.REAR_CAMERA_NAME);

      AprilTagFieldLayout fieldLayout = FieldConfiguration.getFieldLayout(VisionConstants.FIELD_MODE);

      Transform3d robotToRearCamera = new Transform3d(
          new Translation3d(
              VisionConstants.REAR_CAMERA_X_OFFSET,
              VisionConstants.REAR_CAMERA_Y_OFFSET,
              VisionConstants.REAR_CAMERA_Z_OFFSET
          ),
          new Rotation3d(
              Math.toRadians(VisionConstants.REAR_CAMERA_ROLL_DEGREES),
              Math.toRadians(VisionConstants.REAR_CAMERA_PITCH_DEGREES),
              Math.toRadians(VisionConstants.REAR_CAMERA_YAW_DEGREES)
          )
      );

      rearEstimator = new PhotonPoseEstimator(
          fieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          robotToRearCamera
      );
      rearEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      m_rearInitialized = true;
      DataLogManager.log("Rear camera initialized: " + VisionConstants.REAR_CAMERA_NAME);
    } catch (Exception e) {
      DriverStation.reportError("Failed to initialize rear camera: " + e.getMessage(), true);
      DataLogManager.log("ERROR: Rear camera initialization failed - " + e.getMessage());
      m_rearInitialized = false;
    }
    m_rearCamera = rearCamera;
    m_rearPoseEstimator = rearEstimator;

    if (m_frontInitialized || m_rearInitialized) {
      DataLogManager.log("VisionSubsystem initialized with " +
          (m_frontInitialized ? "front " : "") +
          (m_rearInitialized ? "rear" : "") + " camera(s)");
    } else {
      DriverStation.reportWarning("VisionSubsystem: No cameras initialized!", false);
    }
  }

  @Override
  public void periodic() {
    // Process front camera
    if (m_frontInitialized) {
      processCamera(m_frontCamera, m_frontPoseEstimator, "Front", true);
    }

    // Process rear camera
    if (m_rearInitialized) {
      processCamera(m_rearCamera, m_rearPoseEstimator, "Rear", false);
    }

    // Overall telemetry
    SmartDashboard.putBoolean("Vision/Front Initialized", m_frontInitialized);
    SmartDashboard.putBoolean("Vision/Rear Initialized", m_rearInitialized);
  }

  /**
   * Process a single camera's vision measurements
   * @param camera The PhotonCamera to process
   * @param poseEstimator The pose estimator for this camera
   * @param cameraName Name for telemetry logging
   * @param isFront Whether this is the front camera (for timestamp tracking)
   */
  private void processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator,
                             String cameraName, boolean isFront) {
    // Get latest camera result
    var results = camera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }
    PhotonPipelineResult result = results.get(results.size() - 1); // Get most recent

    // Publish camera-specific telemetry
    String prefix = "Vision/" + cameraName + "/";
    SmartDashboard.putBoolean(prefix + "Has Targets", result.hasTargets());
    SmartDashboard.putNumber(prefix + "Target Count", result.getTargets().size());
    SmartDashboard.putBoolean(prefix + "Connected", camera.isConnected());

    // Process vision measurement if we have targets
    if (result.hasTargets()) {
      // Update pose estimator with current robot pose
      poseEstimator.setReferencePose(m_drivetrain.getState().Pose);

      // Get estimated pose from vision
      Optional<EstimatedRobotPose> estimatedPose = poseEstimator.update(result);

      if (estimatedPose.isPresent()) {
        EstimatedRobotPose visionPose = estimatedPose.get();

        // Check quality of the estimate
        boolean shouldUse = shouldUseVisionMeasurement(visionPose, result);

        SmartDashboard.putBoolean(prefix + "Using Measurement", shouldUse);
        SmartDashboard.putNumber(prefix + "Estimated X", visionPose.estimatedPose.getX());
        SmartDashboard.putNumber(prefix + "Estimated Y", visionPose.estimatedPose.getY());
        SmartDashboard.putNumber(prefix + "Estimated Rotation",
            visionPose.estimatedPose.getRotation().toRotation2d().getDegrees());

        if (shouldUse) {
          // Calculate dynamic standard deviations based on distance and number of tags
          Matrix<N3, N1> stdDevs = calculateStdDevs(visionPose, result);

          // Add vision measurement to drivetrain
          m_drivetrain.addVisionMeasurement(
              visionPose.estimatedPose.toPose2d(),
              visionPose.timestampSeconds,
              stdDevs
          );

          // Update timestamp tracking
          if (isFront) {
            m_lastFrontEstimateTimestamp = visionPose.timestampSeconds;
          } else {
            m_lastRearEstimateTimestamp = visionPose.timestampSeconds;
          }

          DataLogManager.log(String.format("%s Vision: Updated pose to (%.2f, %.2f, %.1f°) with %d tags",
              cameraName,
              visionPose.estimatedPose.getX(),
              visionPose.estimatedPose.getY(),
              visionPose.estimatedPose.getRotation().toRotation2d().getDegrees(),
              visionPose.targetsUsed.size()));
        }
      }
    }
  }

  /**
   * Determine if vision measurement should be used
   * @param pose The estimated pose from vision
   * @param result The pipeline result
   * @return True if measurement should be used
   */
  private boolean shouldUseVisionMeasurement(EstimatedRobotPose pose, PhotonPipelineResult result) {
    // Don't use if we don't have targets
    if (!result.hasTargets()) {
      return false;
    }

    // Check ambiguity for single-tag detections
    if (pose.targetsUsed.size() == 1) {
      double ambiguity = result.getBestTarget().getPoseAmbiguity();
      if (ambiguity > VisionConstants.MAX_AMBIGUITY) {
        return false;
      }
    }

    // Check distance - vision gets unreliable at long distances
    Pose2d currentPose = m_drivetrain.getState().Pose;
    double distance = currentPose.getTranslation().getDistance(pose.estimatedPose.toPose2d().getTranslation());

    if (distance > VisionConstants.MAX_VISION_DISTANCE) {
      return false;
    }

    return true;
  }

  /**
   * Calculate dynamic standard deviations based on measurement quality
   * @param pose The estimated pose
   * @param result The pipeline result
   * @return Standard deviation matrix [x, y, rotation]
   */
  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose pose, PhotonPipelineResult result) {
    // Start with base standard deviations
    double xyStdDev = VisionConstants.VISION_MEASUREMENT_STD_DEVS[0];
    double rotStdDev = VisionConstants.VISION_MEASUREMENT_STD_DEVS[2];

    // Calculate distance to target
    Pose2d currentPose = m_drivetrain.getState().Pose;
    double distance = currentPose.getTranslation().getDistance(pose.estimatedPose.toPose2d().getTranslation());

    // Increase standard deviation with distance
    double distanceWeight = 1.0 + (distance * VisionConstants.DISTANCE_WEIGHT);
    xyStdDev *= distanceWeight;
    rotStdDev *= distanceWeight;

    // Decrease standard deviation (increase trust) with more tags
    int numTags = pose.targetsUsed.size();
    if (numTags > 1) {
      double tagWeight = 1.0 / Math.sqrt(numTags);
      xyStdDev *= tagWeight;
      rotStdDev *= tagWeight;
    }

    return VecBuilder.fill(xyStdDev, xyStdDev, rotStdDev);
  }

  /**
   * Check if any camera initialized successfully
   * @return True if at least one camera initialized
   */
  public boolean isInitialized() {
    return m_frontInitialized || m_rearInitialized;
  }

  /**
   * Check if front camera initialized
   * @return True if front camera initialized
   */
  public boolean isFrontInitialized() {
    return m_frontInitialized;
  }

  /**
   * Check if rear camera initialized
   * @return True if rear camera initialized
   */
  public boolean isRearInitialized() {
    return m_rearInitialized;
  }

  /**
   * Check if front camera is connected
   * @return True if front camera is connected
   */
  public boolean isFrontCameraConnected() {
    return m_frontInitialized && m_frontCamera != null && m_frontCamera.isConnected();
  }

  /**
   * Check if rear camera is connected
   * @return True if rear camera is connected
   */
  public boolean isRearCameraConnected() {
    return m_rearInitialized && m_rearCamera != null && m_rearCamera.isConnected();
  }

  /**
   * Get the timestamp of the last accepted front camera estimate
   * @return Timestamp in seconds
   */
  public double getLastFrontEstimateTimestamp() {
    return m_lastFrontEstimateTimestamp;
  }

  /**
   * Get the timestamp of the last accepted rear camera estimate
   * @return Timestamp in seconds
   */
  public double getLastRearEstimateTimestamp() {
    return m_lastRearEstimateTimestamp;
  }

  /**
   * Get the nearest AprilTag detected by the front camera
   * @return Optional containing the target data, or empty if no targets detected
   */
  public Optional<org.photonvision.targeting.PhotonTrackedTarget> getNearestFrontTarget() {
    if (!m_frontInitialized) {
      return Optional.empty();
    }

    var results = m_frontCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }
    PhotonPipelineResult result = results.get(results.size() - 1);

    if (!result.hasTargets()) {
      return Optional.empty();
    }

    // Return the best target (PhotonVision already sorts by quality/area)
    return Optional.of(result.getBestTarget());
  }

  /**
   * Get a specific AprilTag by ID from the front camera
   * @param targetId The AprilTag ID to find
   * @return Optional containing the target data, or empty if not found
   */
  public Optional<org.photonvision.targeting.PhotonTrackedTarget> getFrontTargetById(int targetId) {
    if (!m_frontInitialized) {
      return Optional.empty();
    }

    var results = m_frontCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return Optional.empty();
    }
    PhotonPipelineResult result = results.get(results.size() - 1);

    if (!result.hasTargets()) {
      return Optional.empty();
    }

    // Search for the specific tag ID
    for (org.photonvision.targeting.PhotonTrackedTarget target : result.getTargets()) {
      if (target.getFiducialId() == targetId) {
        return Optional.of(target);
      }
    }

    return Optional.empty();
  }

  /**
   * Get the latest result from the front camera
   * @return Latest pipeline result from front camera
   */
  public PhotonPipelineResult getFrontCameraResult() {
    if (!m_frontInitialized || m_frontCamera == null) {
      return new PhotonPipelineResult();
    }
    var results = m_frontCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new PhotonPipelineResult();
    }
    return results.get(results.size() - 1);
  }

  /**
   * Get the latest result from the rear camera
   * @return Latest pipeline result from rear camera
   */
  public PhotonPipelineResult getRearCameraResult() {
    if (!m_rearInitialized || m_rearCamera == null) {
      return new PhotonPipelineResult();
    }
    var results = m_rearCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return new PhotonPipelineResult();
    }
    return results.get(results.size() - 1);
  }
}
