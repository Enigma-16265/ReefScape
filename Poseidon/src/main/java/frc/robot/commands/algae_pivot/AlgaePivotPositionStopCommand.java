package frc.robot.commands.algae_pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

public class AlgaePivotPositionStopCommand extends Command
{
    private final AlgaePivot elevator;
    private final double     targetPosition;
    private final double     tolerance;

    public AlgaePivotPositionStopCommand( AlgaePivot elevator, double targetPosition, double tolerance )
    {
        this.elevator       = elevator;
        this.targetPosition = targetPosition;
        this.tolerance      = tolerance;

        addRequirements( elevator );
    }

    @Override
    public void initialize()
    {
        elevator.setPosition( targetPosition );
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        double error = Math.abs( elevator.getPosition() - targetPosition );
        return error <= tolerance;
    }

    @Override
    public void end( boolean interrupted )
    {
        elevator.setSpeed( 0.0 );
    }
}