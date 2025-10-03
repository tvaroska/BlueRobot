// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralMechanism;

/**
 * Autonomous selector for choosing between different auto routines
 */
public class AutoSelector {
  private static SendableChooser<Command> chooser = new SendableChooser<>();

  /**
   * Initialize Algae robot autonomous options
   */
  public static Command initializeAlgaeAutos(CommandSwerveDrivetrain drivetrain,
                                             VisionSubsystem vision,
                                             Pneumatics pneumatics) {
    chooser.setDefaultOption("Do Nothing", Commands.none());
    chooser.addOption("Drive to Tag 1", DriveToAprilTag.toTag(drivetrain, vision, 1));
    chooser.addOption("Drive to Nearest Tag", new DriveToAprilTag(drivetrain, vision));

    SmartDashboard.putData("Auto Selector", chooser);
    return getSelectedAuto();
  }

  /**
   * Initialize Coral robot autonomous options
   */
  public static Command initializeCoralAutos(CommandSwerveDrivetrain drivetrain,
                                             VisionSubsystem vision,
                                             ElevatorSubsystem elevator,
                                             CoralMechanism coralMech) {
    chooser.setDefaultOption("Do Nothing", Commands.none());
    chooser.addOption("Drive to Tag 1", DriveToAprilTag.toTag(drivetrain, vision, 1));
    chooser.addOption("Drive to Nearest Tag", new DriveToAprilTag(drivetrain, vision));

    SmartDashboard.putData("Auto Selector", chooser);
    return getSelectedAuto();
  }

  /**
   * Get the selected autonomous command
   */
  public static Command getSelectedAuto() {
    return chooser.getSelected();
  }
}
