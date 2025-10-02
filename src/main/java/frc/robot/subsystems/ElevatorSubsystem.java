package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase {
  private final SparkMax motor;
  private final SparkMaxConfig motorConfig;
  private final SparkLimitSwitch forwardLimitSwitch;
  private final SparkLimitSwitch reverseLimitSwitch;
  private final RelativeEncoder encoder;
  private final SparkClosedLoopController closedLoop;
  private final ClosedLoopConfig pidConfig;

  private double kP = 0.05;
  private double kI = 0.0;
  private double kD = 0.0;

  private double targetPosition = 0.0;

  public ElevatorSubsystem() {
    motor = new SparkMax(1, MotorType.kBrushless);
    forwardLimitSwitch = motor.getForwardLimitSwitch();
    reverseLimitSwitch = motor.getReverseLimitSwitch();
    encoder = motor.getEncoder();
    closedLoop = motor.getClosedLoopController();

    // Motor config
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);

    // Limit switches
    motorConfig.limitSwitch
        .forwardLimitSwitchType(Type.kNormallyOpen)
        .forwardLimitSwitchEnabled(true)
        .reverseLimitSwitchType(Type.kNormallyOpen)
        .reverseLimitSwitchEnabled(true);

    // Soft limits
    motorConfig.softLimit
        .forwardSoftLimit(500)
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimit(-500)
        .reverseSoftLimitEnabled(true);

    // PID
    pidConfig = motorConfig.closedLoop;
    pidConfig.p(kP);
    pidConfig.i(kI);
    pidConfig.d(kD);
    pidConfig.outputRange(-0.3, 0.3);

    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    encoder.setPosition(0);

    SmartDashboard.putNumber("kP", kP);
    SmartDashboard.putNumber("kI", kI);
    SmartDashboard.putNumber("kD", kD);
  }

  @Override
  public void periodic() {
    // Live PID tuning
    double newP = SmartDashboard.getNumber("kP", kP);
    double newI = SmartDashboard.getNumber("kI", kI);
    double newD = SmartDashboard.getNumber("kD", kD);

    if (newP != kP || newI != kI || newD != kD) {
      kP = newP;
      kI = newI;
      kD = newD;
      pidConfig.p(kP);
      pidConfig.i(kI);
      pidConfig.d(kD);
      motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    SmartDashboard.putNumber("Elevator Position", encoder.getPosition());
    SmartDashboard.putNumber("Target Position", targetPosition);
    SmartDashboard.putNumber("Motor Output", motor.getAppliedOutput());
  }

  // ---- Control methods ----
  public void jogUp() {
    motor.set(0.2);
  }

  public void jogDown() {
    motor.set(-0.2);
  }

  public void stop() {
    motor.set(0.0);
  }

  public void moveToTop() {
    targetPosition = -19.5;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }

  public void moveToL1() {
    targetPosition = -8.5;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }

  public void moveToBottom() {
    targetPosition = -0.5;
    closedLoop.setReference(targetPosition, SparkMax.ControlType.kPosition);
  }
}
