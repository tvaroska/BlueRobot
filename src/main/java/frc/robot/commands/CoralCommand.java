package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.CoralMechanism;

public class CoralCommand extends Command {
    private final CoralMechanism coralMech;
    private final DoubleSupplier speed;
    private boolean into = true;

    private boolean coralInside = false;
    enum CoralStage {
        In,
        Inside,
        Out
    }
    private CoralStage stage = CoralStage.In;

    public CoralCommand(CoralMechanism coral, DoubleSupplier speed, boolean into)
    {
        coralMech = coral;
        this.speed = speed;
        this.into = into;
        addRequirements(coral);
    }

    @Override
    public void initialize() {
        stage = CoralStage.In;
    }
  
    @Override
    public void execute() {
        if (stage == CoralStage.In)
            coralMech.setSpeed(speed.getAsDouble(), into);
    }
  
    @Override
    public void end(boolean interrupted) {
        coralMech.setSpeed(0, true);
    }
  
    @Override
    public boolean isFinished() {
        return false;//coralMech.isCoralIn();
    }
}
