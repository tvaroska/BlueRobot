package frc.robot.generated;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class ModuleConstants {
    public final int driveMotorId;
    public final int steerMotorId;
    public final int encoderId;
    public final boolean steerMotorInverted;
    public final boolean encoderInverted;
    public final Angle encoderOffset;

    public final Distance xPos;
    public final Distance yPos;

    public ModuleConstants(int driveMotorId, int steerMotorId, int encoderId,
                           double encoderOffsetAngle, double xPos, double yPos,
                           boolean steerMotorInverted, boolean encoderInverted) {
        this.driveMotorId = driveMotorId;
        this.steerMotorId = steerMotorId;
        this.encoderId = encoderId;
        this.encoderOffset = Rotations.of(encoderOffsetAngle);
        this.steerMotorInverted = steerMotorInverted;
        this.encoderInverted = encoderInverted;
        this.xPos = Inches.of(xPos);
        this.yPos = Inches.of(yPos);
    }
}

