// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.subsystems.CoralMechanism;
import frc.robot.commands.CoralCommand;
import frc.robot.commands.AutoSelector;

public class CoralRobotContainer extends RobotContainer {
    private final CoralMechanism coralMech = new CoralMechanism(TunerConstants.kCANBus);

    public final ElevatorSubsystem elevator = new ElevatorSubsystem();

    public CoralRobotContainer() {
        super(RobotConstants.CoralRobot.FrontLeft, RobotConstants.CoralRobot.FrontRight,
            RobotConstants.CoralRobot.BackLeft, RobotConstants.CoralRobot.BackRight);
        configureBindings();
    }

    @Override
    protected void configureBindings() {
        super.configureBindings();

        // Coral Mechanism
        // Run Coral motor based on right trigger pressure
        // coralMech.setDefaultCommand(
        //     new RunCommand(
        //         () -> {
        //             double forward = joystick.getRightTriggerAxis(); // 0 → 1
        //             double reverse = joystick.getLeftTriggerAxis();  // 0 → 1
        //             double speed = forward - reverse; // right = positive, left = negative
        //             coralMech.setSpeed(speed, true);
        //         },
        //         coralMech
        //     )
        // );

        // OR leftTrigger(0.0).onTrue
        joystick.rightTrigger().whileTrue(new CoralCommand(coralMech, () -> joystick.getRightTriggerAxis(), true));
        joystick.leftTrigger().whileTrue(new CoralCommand(coralMech, () -> joystick.getLeftTriggerAxis(), false));


        //Elevator bindings
        joystick.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
        joystick.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
        joystick.povLeft().or(joystick.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

        joystick.y().onTrue(new RunCommand(() -> elevator.moveToTop(), elevator));
        joystick.b().onTrue(new RunCommand(() -> elevator.moveToL1(), elevator));
        joystick.a().onTrue(new RunCommand(() -> elevator.moveToBottom(), elevator));
    }

    @Override
    public Command getAutonomousCommand() {
        return AutoSelector.initializeCoralAutos(drivetrain, getVisionSubsystem(), elevator, coralMech);
    }
}
