package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.FieldConfiguration;

public final class VisionConstants {
    // Camera names (must match PhotonVision config)
    public static final String FRONT_CAMERA_NAME = "front_camera";
    public static final String REAR_CAMERA_NAME = "rear_camera";

    // Front camera configuration
    public static final double FRONT_CAMERA_X_OFFSET = 0.3; // 0.3m forward from center
    public static final double FRONT_CAMERA_Y_OFFSET = 0.0; // Centered left/right
    public static final double FRONT_CAMERA_Z_OFFSET = 0.5; // 0.5m height above ground
    public static final double FRONT_CAMERA_ROLL_DEGREES = 0.0;
    public static final double FRONT_CAMERA_PITCH_DEGREES = 0.0;
    public static final double FRONT_CAMERA_YAW_DEGREES = 0.0; // Facing forward

    // Front camera transform
    public static final Transform3d FRONT_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(FRONT_CAMERA_X_OFFSET, FRONT_CAMERA_Y_OFFSET, FRONT_CAMERA_Z_OFFSET),
        new Rotation3d(
            Math.toRadians(FRONT_CAMERA_ROLL_DEGREES),
            Math.toRadians(FRONT_CAMERA_PITCH_DEGREES),
            Math.toRadians(FRONT_CAMERA_YAW_DEGREES)
        )
    );

    // Rear camera configuration
    public static final double REAR_CAMERA_X_OFFSET = -0.3; // 0.3m behind center
    public static final double REAR_CAMERA_Y_OFFSET = 0.0; // Centered left/right
    public static final double REAR_CAMERA_Z_OFFSET = 0.5; // 0.5m height above ground
    public static final double REAR_CAMERA_ROLL_DEGREES = 0.0;
    public static final double REAR_CAMERA_PITCH_DEGREES = 0.0;
    public static final double REAR_CAMERA_YAW_DEGREES = 180.0; // Facing backward

    // Rear camera transform
    public static final Transform3d REAR_CAMERA_TRANSFORM = new Transform3d(
        new Translation3d(REAR_CAMERA_X_OFFSET, REAR_CAMERA_Y_OFFSET, REAR_CAMERA_Z_OFFSET),
        new Rotation3d(
            Math.toRadians(REAR_CAMERA_ROLL_DEGREES),
            Math.toRadians(REAR_CAMERA_PITCH_DEGREES),
            Math.toRadians(REAR_CAMERA_YAW_DEGREES)
        )
    );

    // Vision measurement standard deviations (trust levels)
    public static final double[] VISION_MEASUREMENT_STD_DEVS = {0.5, 0.5, 0.5};

    // Distance-based standard deviation scaling
    public static final double DISTANCE_WEIGHT = 2.0; // Multiplier for distance effect

    // Maximum distance to trust vision measurements (meters)
    public static final double MAX_VISION_DISTANCE = 4.0;

    // Ambiguity threshold (0-1, lower is better)
    public static final double MAX_AMBIGUITY = 0.3;

    // Field mode
    public static final FieldConfiguration.FieldMode FIELD_MODE =
        FieldConfiguration.FieldMode.REAL_FIELD;

    // Training Field Configuration
    public static final double TRAINING_FIELD_LENGTH = 8.0; // meters
    public static final double TRAINING_FIELD_WIDTH = 6.0; // meters

    // Training Tag 1 - Center position
    public static final int TRAINING_TAG_1_ID = 1;
    public static final double TRAINING_TAG_1_X = 4.0;
    public static final double TRAINING_TAG_1_Y = 3.0;
    public static final double TRAINING_TAG_1_Z = 1.45;
    public static final double TRAINING_TAG_1_ROLL_DEGREES = 0.0;
    public static final double TRAINING_TAG_1_PITCH_DEGREES = 0.0;
    public static final double TRAINING_TAG_1_YAW_DEGREES = 180.0;

    // Training Tag 2 - Left position
    public static final int TRAINING_TAG_2_ID = 2;
    public static final double TRAINING_TAG_2_X = 4.0;
    public static final double TRAINING_TAG_2_Y = 1.0;
    public static final double TRAINING_TAG_2_Z = 1.45;
    public static final double TRAINING_TAG_2_ROLL_DEGREES = 0.0;
    public static final double TRAINING_TAG_2_PITCH_DEGREES = 0.0;
    public static final double TRAINING_TAG_2_YAW_DEGREES = 180.0;

    // Training Tag 3 - Right position
    public static final int TRAINING_TAG_3_ID = 3;
    public static final double TRAINING_TAG_3_X = 4.0;
    public static final double TRAINING_TAG_3_Y = 5.0;
    public static final double TRAINING_TAG_3_Z = 1.45;
    public static final double TRAINING_TAG_3_ROLL_DEGREES = 0.0;
    public static final double TRAINING_TAG_3_PITCH_DEGREES = 0.0;
    public static final double TRAINING_TAG_3_YAW_DEGREES = 180.0;

    // Safety thresholds
    public static final class Safety {
        public static final double VOLTAGE_WARNING_THRESHOLD = 11.5;
        public static final double VOLTAGE_CRITICAL_THRESHOLD = 10.5;
        public static final double CURRENT_WARNING_THRESHOLD = 200.0;
        public static final double CURRENT_CRITICAL_THRESHOLD = 250.0;
        public static final double CAN_UTILIZATION_WARNING = 90.0;
        public static final double WARNING_THROTTLE_SECONDS = 5.0;
    }

    // Auto drive to tag PID constants
    public static final class DriveToTag {
        public static final double APRILTAG_DISTANCE_METERS = 0.508; // 20 inches = 0.508 meters
        public static final double APRILTAG_POSITION_TOLERANCE = 0.05; // 5cm distance tolerance
        public static final double APRILTAG_ROTATION_TOLERANCE = 5.0; // 5 degrees yaw tolerance
        public static final double APRILTAG_STRAFE_TOLERANCE = 2.0; // degrees yaw tolerance for centering
        public static final double APRILTAG_MAX_SPEED = 1.0; // m/s max approach speed
        public static final double APRILTAG_MAX_ROTATION_SPEED = 1.0; // rad/s max rotation speed

        // PID tuning constants [kP, kI, kD]
        public static final double[] APRILTAG_FORWARD_PID = {1.5, 0.0, 0.1}; // Forward/backward control
        public static final double[] APRILTAG_STRAFE_PID = {0.05, 0.0, 0.005}; // Left/right centering
        public static final double[] APRILTAG_ROTATION_PID = {0.08, 0.0, 0.01}; // Rotation to face tag
    }
}
