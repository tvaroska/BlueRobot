package frc.robot;

import frc.robot.generated.ModuleConstants;

public class RobotConstants {
    public ModuleConstants FrontLeft;
    public ModuleConstants FrontRight;
    public ModuleConstants BackLeft;
    public ModuleConstants BackRight;

    public static RobotConstants CoralRobot;
    public static RobotConstants AlgaeRobot;

    public RobotConstants() {
        CoralRobot.FrontLeft = new ModuleConstants(7, 8, 23, 0.124267578125, 11.5, 11.5, false, false);
        CoralRobot.FrontRight = new ModuleConstants(1, 2, 20, -0.291015625, 11.5, -11.5, false, false);
        CoralRobot.BackLeft = new ModuleConstants(5, 6, 22, 0.048828125, -11.5, 11.5, false, false);
        CoralRobot.BackRight = new ModuleConstants(3, 4, 21, -0.371826171875, -11.5, -11.5, false, false);

        AlgaeRobot.FrontLeft = new ModuleConstants(7, 8, 23, 0.124267578125, 11.5, 11.5, false, false);
        AlgaeRobot.FrontRight = new ModuleConstants(1, 2, 20, -0.291015625, 11.5, -11.5, false, false);
        AlgaeRobot.BackLeft = new ModuleConstants(5, 6, 22, 0.048828125, -11.5, 11.5, false, false);
        AlgaeRobot.BackRight = new ModuleConstants(3, 4, 21, -0.371826171875, -11.5, -11.5, false, false);
    }
}
