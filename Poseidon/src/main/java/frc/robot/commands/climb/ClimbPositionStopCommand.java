package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbPositionStopCommand extends Command
{
    private final Climb climb;
    private final double     targetPosition;
    private final double     tolerance;

    public ClimbPositionStopCommand( Climb climb, double targetPosition, double tolerance )
    {
        this.climb       = climb;
        this.targetPosition = targetPosition;
        this.tolerance      = tolerance;

        addRequirements( climb );
    }

    @Override
    public void initialize()
    {
        climb.setPosition( targetPosition );
    }

    @Override
    public void execute()
    {

    }

    @Override
    public boolean isFinished()
    {
        double error = Math.abs( climb.getPosition() - targetPosition );
        return error <= tolerance;
    }

    @Override
    public void end( boolean interrupted )
    {
        climb.setSpeed( 0.0 );
    }
}
