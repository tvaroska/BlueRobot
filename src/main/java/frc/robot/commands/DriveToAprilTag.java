// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to autonomously drive to an AprilTag detected by the front camera
 * Uses vision feedback to center the robot on the tag and maintain specified distance
 * Can target a specific tag ID or the nearest visible tag
 */
public class DriveToAprilTag extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final VisionSubsystem m_visionSubsystem;
  private final double m_targetDistance;
  private final int m_targetTagId; // -1 = nearest tag, otherwise specific ID

  private final PIDController m_forwardController;
  private final PIDController m_strafeController;
  private final PIDController m_rotationController;

  private final SwerveRequest.RobotCentric m_driveRequest;
  private PhotonTrackedTarget m_currentTarget;
  private PhotonPipelineResult m_lastResult;

  /**
   * Creates a DriveToAprilTag command with all parameters
   * @param drivetrain The swerve drivetrain
   * @param visionSubsystem The vision subsystem for AprilTag detection
   * @param targetTagId The AprilTag ID to target (-1 for nearest tag)
   * @param targetDistanceMeters Distance to maintain from tag (meters)
   */
  public DriveToAprilTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem,
                         int targetTagId, double targetDistanceMeters) {
    m_drivetrain = drivetrain;
    m_visionSubsystem = visionSubsystem;
    m_targetTagId = targetTagId;
    m_targetDistance = targetDistanceMeters;
    m_currentTarget = null;

    // PID controllers - using robot-centric control based on vision angles/distances
    // Forward/backward to achieve target distance
    m_forwardController = new PIDController(
        VisionConstants.DriveToTag.APRILTAG_FORWARD_PID[0],
        VisionConstants.DriveToTag.APRILTAG_FORWARD_PID[1],
        VisionConstants.DriveToTag.APRILTAG_FORWARD_PID[2]
    );
    // Left/right to center on tag (yaw angle)
    m_strafeController = new PIDController(
        VisionConstants.DriveToTag.APRILTAG_STRAFE_PID[0],
        VisionConstants.DriveToTag.APRILTAG_STRAFE_PID[1],
        VisionConstants.DriveToTag.APRILTAG_STRAFE_PID[2]
    );
    // Rotation to face tag directly
    m_rotationController = new PIDController(
        VisionConstants.DriveToTag.APRILTAG_ROTATION_PID[0],
        VisionConstants.DriveToTag.APRILTAG_ROTATION_PID[1],
        VisionConstants.DriveToTag.APRILTAG_ROTATION_PID[2]
    );

    // Rotation controller wraps around at +/- 180 degrees
    m_rotationController.enableContinuousInput(-180, 180);

    // Set tolerances
    m_forwardController.setTolerance(VisionConstants.DriveToTag.APRILTAG_POSITION_TOLERANCE);
    m_strafeController.setTolerance(VisionConstants.DriveToTag.APRILTAG_STRAFE_TOLERANCE);
    m_rotationController.setTolerance(VisionConstants.DriveToTag.APRILTAG_ROTATION_TOLERANCE);

    m_driveRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withDeadband(0.0)
        .withRotationalDeadband(0.0);

    addRequirements(drivetrain);
  }

  /**
   * Creates a DriveToAprilTag command targeting nearest tag with default distance
   * @param drivetrain The swerve drivetrain
   * @param visionSubsystem The vision subsystem
   */
  public DriveToAprilTag(CommandSwerveDrivetrain drivetrain, VisionSubsystem visionSubsystem) {
    this(drivetrain, visionSubsystem, -1, VisionConstants.DriveToTag.APRILTAG_DISTANCE_METERS);
  }

  /**
   * Creates a DriveToAprilTag command targeting a specific tag with default distance
   * @param drivetrain The swerve drivetrain
   * @param visionSubsystem The vision subsystem
   * @param targetTagId The AprilTag ID to target
   */
  public static DriveToAprilTag toTag(CommandSwerveDrivetrain drivetrain,
                                       VisionSubsystem visionSubsystem,
                                       int targetTagId) {
    return new DriveToAprilTag(drivetrain, visionSubsystem, targetTagId,
                               VisionConstants.DriveToTag.APRILTAG_DISTANCE_METERS);
  }

  /**
   * Creates a DriveToAprilTag command with custom tag and distance
   * @param drivetrain The swerve drivetrain
   * @param visionSubsystem The vision subsystem
   * @param targetTagId The AprilTag ID to target
   * @param distanceInches Distance in inches from tag
   */
  public static DriveToAprilTag toTag(CommandSwerveDrivetrain drivetrain,
                                       VisionSubsystem visionSubsystem,
                                       int targetTagId,
                                       double distanceInches) {
    return new DriveToAprilTag(drivetrain, visionSubsystem, targetTagId,
                               distanceInches * 0.0254); // Convert inches to meters
  }

  @Override
  public void initialize() {
    if (m_targetTagId == -1) {
      DataLogManager.log("DriveToAprilTag started: Targeting nearest tag at " + m_targetDistance + "m using vision");
    } else {
      DataLogManager.log("DriveToAprilTag started: Targeting tag " + m_targetTagId +
                         " at " + m_targetDistance + "m using vision");
    }

    // Reset PID controllers
    m_forwardController.reset();
    m_strafeController.reset();
    m_rotationController.reset();

    m_currentTarget = null;
    m_lastResult = null;
  }

  @Override
  public void execute() {
    // Get the latest camera result (cached to avoid creating new objects)
    m_lastResult = m_visionSubsystem.getFrontCameraResult();

    if (!m_lastResult.hasTargets()) {
      // No target visible, stop
      m_drivetrain.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      m_currentTarget = null;
      return;
    }

    // Find the target (either specific ID or nearest)
    PhotonTrackedTarget target = null;

    if (m_targetTagId == -1) {
      // Target nearest tag (best target)
      target = m_lastResult.getBestTarget();
    } else {
      // Target specific tag by ID
      for (PhotonTrackedTarget t : m_lastResult.getTargets()) {
        if (t.getFiducialId() == m_targetTagId) {
          target = t;
          break;
        }
      }
    }

    if (target == null) {
      // Specific tag not found, stop
      m_drivetrain.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      m_currentTarget = null;
      return;
    }

    m_currentTarget = target;

    // Get target data from PhotonVision
    // Yaw: horizontal angle to target (positive = target to the right)
    // Pitch: vertical angle to target
    // Area: percentage of image occupied by target
    double yaw = m_currentTarget.getYaw();
    double pitch = m_currentTarget.getPitch();

    // Estimate distance using camera pitch and known camera height
    // This is approximate - works best when camera height and tag height are known
    // For now, use the target's best camera-to-target transform if available
    double estimatedDistance = 0.0;
    if (m_currentTarget.getBestCameraToTarget().getTranslation().getNorm() > 0) {
      estimatedDistance = m_currentTarget.getBestCameraToTarget().getTranslation().getNorm();
    }

    // Calculate control outputs
    // Forward speed: close the distance to target distance
    double distanceError = estimatedDistance - m_targetDistance;
    double forwardSpeed = m_forwardController.calculate(0, -distanceError);

    // Strafe speed: center on target (eliminate yaw)
    double strafeSpeed = m_strafeController.calculate(0, -yaw);

    // Rotation speed: face the tag (yaw should be zero when centered)
    double rotSpeed = m_rotationController.calculate(0, -yaw);

    // Limit speeds
    forwardSpeed = Math.max(-VisionConstants.DriveToTag.APRILTAG_MAX_SPEED, Math.min(VisionConstants.DriveToTag.APRILTAG_MAX_SPEED, forwardSpeed));
    strafeSpeed = Math.max(-VisionConstants.DriveToTag.APRILTAG_MAX_SPEED, Math.min(VisionConstants.DriveToTag.APRILTAG_MAX_SPEED, strafeSpeed));
    rotSpeed = Math.max(-VisionConstants.DriveToTag.APRILTAG_MAX_ROTATION_SPEED, Math.min(VisionConstants.DriveToTag.APRILTAG_MAX_ROTATION_SPEED, rotSpeed));

    // Apply control to drivetrain (robot-centric)
    m_drivetrain.setControl(
        m_driveRequest
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(rotSpeed)
    );
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_drivetrain.setControl(m_driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0));

    if (interrupted) {
      DataLogManager.log("DriveToAprilTag interrupted");
    } else {
      DataLogManager.log("DriveToAprilTag completed - arrived at target");
    }
  }

  @Override
  public boolean isFinished() {
    // This command runs continuously while the button is held
    // Never finishes on its own - only when button is released
    return false;
  }
}
