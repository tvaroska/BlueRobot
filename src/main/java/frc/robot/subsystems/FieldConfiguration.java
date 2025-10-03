// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.VisionConstants;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Utility class for managing field configurations
 * Supports both the official 2025 Reefscape field and a training field with a single AprilTag
 */
public class FieldConfiguration {

  public enum FieldMode {
    /** Official 2025 FRC Reefscape field with all 16 AprilTags */
    REAL_FIELD,

    /** Training field with a single AprilTag for basic testing */
    TRAINING_FIELD_1_TAG,

    /** Training field with two AprilTags for alignment and multi-tag testing */
    TRAINING_FIELD_2_TAGS,

    /** Training field with three AprilTags for full pose estimation testing */
    TRAINING_FIELD_3_TAGS
  }

  /**
   * Get the AprilTag field layout based on the configured mode
   * @param mode The field mode to use
   * @return AprilTagFieldLayout for the specified mode
   */
  public static AprilTagFieldLayout getFieldLayout(FieldMode mode) {
    switch (mode) {
      case REAL_FIELD:
        return getRealFieldLayout();
      case TRAINING_FIELD_1_TAG:
        return getTrainingFieldLayout(1);
      case TRAINING_FIELD_2_TAGS:
        return getTrainingFieldLayout(2);
      case TRAINING_FIELD_3_TAGS:
        return getTrainingFieldLayout(3);
      default:
        DataLogManager.log("WARNING: Unknown field mode, defaulting to REAL_FIELD");
        return getRealFieldLayout();
    }
  }

  /**
   * Get the official 2025 Reefscape field layout
   * @return AprilTagFieldLayout with all 16 tags
   */
  private static AprilTagFieldLayout getRealFieldLayout() {
    // Load the 2025 field layout
    // Note: Field name may vary by WPILib version (k2025Reefscape, kDefaultField, etc.)
    AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    DataLogManager.log("Loaded 2025 field layout with " + layout.getTags().size() + " AprilTags");
    return layout;
  }

  /**
   * Get a training field layout with configurable number of AprilTags
   * Uses configurable positions from Constants
   * @param numTags Number of tags to include (1, 2, or 3)
   * @return AprilTagFieldLayout with specified number of tags
   */
  private static AprilTagFieldLayout getTrainingFieldLayout(int numTags) {
    List<AprilTag> tags = new ArrayList<>();

    // Tag 1 - Always included (center position)
    if (numTags >= 1) {
      tags.add(new AprilTag(
          VisionConstants.TRAINING_TAG_1_ID,
          new Pose3d(
              new Translation3d(
                  VisionConstants.TRAINING_TAG_1_X,
                  VisionConstants.TRAINING_TAG_1_Y,
                  VisionConstants.TRAINING_TAG_1_Z
              ),
              new Rotation3d(
                  Math.toRadians(VisionConstants.TRAINING_TAG_1_ROLL_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_1_PITCH_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_1_YAW_DEGREES)
              )
          )
      ));
    }

    // Tag 2 - Left side position
    if (numTags >= 2) {
      tags.add(new AprilTag(
          VisionConstants.TRAINING_TAG_2_ID,
          new Pose3d(
              new Translation3d(
                  VisionConstants.TRAINING_TAG_2_X,
                  VisionConstants.TRAINING_TAG_2_Y,
                  VisionConstants.TRAINING_TAG_2_Z
              ),
              new Rotation3d(
                  Math.toRadians(VisionConstants.TRAINING_TAG_2_ROLL_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_2_PITCH_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_2_YAW_DEGREES)
              )
          )
      ));
    }

    // Tag 3 - Right side position
    if (numTags >= 3) {
      tags.add(new AprilTag(
          VisionConstants.TRAINING_TAG_3_ID,
          new Pose3d(
              new Translation3d(
                  VisionConstants.TRAINING_TAG_3_X,
                  VisionConstants.TRAINING_TAG_3_Y,
                  VisionConstants.TRAINING_TAG_3_Z
              ),
              new Rotation3d(
                  Math.toRadians(VisionConstants.TRAINING_TAG_3_ROLL_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_3_PITCH_DEGREES),
                  Math.toRadians(VisionConstants.TRAINING_TAG_3_YAW_DEGREES)
              )
          )
      ));
    }

    // Use standard field dimensions (or smaller for training)
    AprilTagFieldLayout layout = new AprilTagFieldLayout(
        tags,
        VisionConstants.TRAINING_FIELD_LENGTH,
        VisionConstants.TRAINING_FIELD_WIDTH
    );

    DataLogManager.log(String.format(
        "Created training field layout with %d tag(s)", numTags
    ));

    return layout;
  }

  /**
   * Print all AprilTags in a field layout to console
   * Useful for debugging and visualization
   * @param layout The field layout to print
   */
  public static void printFieldLayout(AprilTagFieldLayout layout) {
    System.out.println("\n=== AprilTag Field Layout ===");
    System.out.println("Field Size: " + layout.getFieldLength() + "m x " + layout.getFieldWidth() + "m");
    System.out.println("Number of Tags: " + layout.getTags().size());
    System.out.println("\nTag Positions:");

    for (AprilTag tag : layout.getTags()) {
      Pose3d pose = tag.pose;
      System.out.printf(
          "  Tag %2d: X=%6.2fm Y=%6.2fm Z=%6.2fm | Yaw=%6.1f° Pitch=%6.1f° Roll=%6.1f°\n",
          tag.ID,
          pose.getX(),
          pose.getY(),
          pose.getZ(),
          pose.getRotation().getZ() * 180.0 / Math.PI,
          pose.getRotation().getY() * 180.0 / Math.PI,
          pose.getRotation().getX() * 180.0 / Math.PI
      );
    }
    System.out.println("=============================\n");
  }

  /**
   * Get the current field mode from Constants
   * @return The configured field mode
   */
  public static FieldMode getCurrentMode() {
    return VisionConstants.FIELD_MODE;
  }
}
