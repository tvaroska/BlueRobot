// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VisionConstants;

/**
 * Safety monitoring subsystem that tracks robot health metrics
 * and reports warnings/errors for voltage, current, and temperature issues
 */
public class SafetyMonitor extends SubsystemBase {
  private double m_lastWarningTime = 0.0;
  private boolean m_brownoutWarningActive = false;
  private boolean m_currentWarningActive = false;

  // Peak tracking
  private double m_peakCurrent = 0.0;
  private double m_lowestVoltage = 13.0;

  public SafetyMonitor() {
    DataLogManager.log("SafetyMonitor subsystem initialized");
  }

  @Override
  public void periodic() {
    double currentTime = Timer.getFPGATimestamp();

    // Get current measurements
    double batteryVoltage = RobotController.getBatteryVoltage();
    double totalCurrent = RobotController.getInputCurrent();
    boolean brownedOut = RobotController.isBrownedOut();

    // Update peaks
    if (totalCurrent > m_peakCurrent) {
      m_peakCurrent = totalCurrent;
    }
    if (batteryVoltage < m_lowestVoltage && batteryVoltage > 0) {
      m_lowestVoltage = batteryVoltage;
    }

    // Publish telemetry
    SmartDashboard.putNumber("Safety/Battery Voltage", batteryVoltage);
    SmartDashboard.putNumber("Safety/Total Current", totalCurrent);
    SmartDashboard.putNumber("Safety/Peak Current", m_peakCurrent);
    SmartDashboard.putNumber("Safety/Lowest Voltage", m_lowestVoltage);
    SmartDashboard.putBoolean("Safety/Browned Out", brownedOut);
    SmartDashboard.putBoolean("Safety/System Active", RobotController.isSysActive());

    // Check for brownout condition
    if (brownedOut) {
      if (!m_brownoutWarningActive) {
        DriverStation.reportError("BROWNOUT DETECTED - Battery critically low!", false);
        DataLogManager.log("CRITICAL: Brownout detected at " + batteryVoltage + "V");
        m_brownoutWarningActive = true;
      }
    } else {
      m_brownoutWarningActive = false;
    }

    // Check voltage thresholds (throttle warnings to every 5 seconds)
    if (currentTime - m_lastWarningTime > 5.0) {
      if (batteryVoltage < VisionConstants.Safety.VOLTAGE_CRITICAL_THRESHOLD) {
        DriverStation.reportError(
            String.format("CRITICAL: Battery voltage critically low! (%.2fV)", batteryVoltage),
            false
        );
        DataLogManager.log("CRITICAL: Voltage at " + batteryVoltage + "V");
        m_lastWarningTime = currentTime;
      } else if (batteryVoltage < VisionConstants.Safety.VOLTAGE_WARNING_THRESHOLD) {
        DriverStation.reportWarning(
            String.format("WARNING: Battery voltage low (%.2fV)", batteryVoltage),
            false
        );
        m_lastWarningTime = currentTime;
      }

      // Check current thresholds
      if (totalCurrent > VisionConstants.Safety.CURRENT_CRITICAL_THRESHOLD) {
        DriverStation.reportError(
            String.format("CRITICAL: Total current extremely high! (%.1fA)", totalCurrent),
            false
        );
        DataLogManager.log("CRITICAL: Current at " + totalCurrent + "A");
        m_lastWarningTime = currentTime;
        m_currentWarningActive = true;
      } else if (totalCurrent > VisionConstants.Safety.CURRENT_WARNING_THRESHOLD) {
        if (!m_currentWarningActive) {
          DriverStation.reportWarning(
              String.format("WARNING: Total current high (%.1fA)", totalCurrent),
              false
          );
          m_currentWarningActive = true;
        }
        m_lastWarningTime = currentTime;
      } else {
        m_currentWarningActive = false;
      }
    }

    // Check CAN bus utilization
    double canUtilization = RobotController.getCANStatus().percentBusUtilization;
    SmartDashboard.putNumber("Safety/CAN Utilization %", canUtilization);

    if (canUtilization > 90.0 && currentTime - m_lastWarningTime > 10.0) {
      DriverStation.reportWarning(
          String.format("WARNING: CAN bus utilization high (%.1f%%)", canUtilization),
          false
      );
      DataLogManager.log("WARNING: CAN utilization at " + canUtilization + "%");
      m_lastWarningTime = currentTime;
    }
  }

  /**
   * Reset peak tracking values
   */
  public void resetPeaks() {
    m_peakCurrent = 0.0;
    m_lowestVoltage = RobotController.getBatteryVoltage();
    DataLogManager.log("Safety peaks reset");
  }

  /**
   * Get current battery voltage
   * @return Battery voltage in volts
   */
  public double getBatteryVoltage() {
    return RobotController.getBatteryVoltage();
  }

  /**
   * Get total current draw
   * @return Total current in amps
   */
  public double getTotalCurrent() {
    return RobotController.getInputCurrent();
  }

  /**
   * Check if robot is currently browned out
   * @return True if browned out
   */
  public boolean isBrownedOut() {
    return RobotController.isBrownedOut();
  }

  /**
   * Get peak current since last reset
   * @return Peak current in amps
   */
  public double getPeakCurrent() {
    return m_peakCurrent;
  }

  /**
   * Get lowest voltage since last reset
   * @return Lowest voltage in volts
   */
  public double getLowestVoltage() {
    return m_lowestVoltage;
  }
}
