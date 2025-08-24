package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralMechanism extends SubsystemBase {
  private final SparkMax motor;
  private boolean motorOn = false;

  public CoralMechanism() {
    motor = new SparkMax(3, MotorType.kBrushless); // CAN ID 3

    // Add a SmartDashboard slider for manual control
    SmartDashboard.putNumber("Coral Motor Speed", 0.0);
  }

  public void setSpeed(double speed) {
    motor.set(speed);
    SmartDashboard.putNumber("Coral Motor Applied Output", motor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // Optional: allow manual override from dashboard
    double manualSpeed = SmartDashboard.getNumber("Coral Motor Speed", 0.0);
    if (manualSpeed != 0.0) {
      motor.set(manualSpeed);
    }
  }
}
