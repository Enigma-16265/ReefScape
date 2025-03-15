package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPositionStopCommand extends Command
{
    private final Elevator elevator;
    private final double   targetPosition;
    private final double   tolerance;

    /**
     * Creates a new command that drives the elevator to a target position 
     * and then stops the PID command once the position is within tolerance.
     *
     * @param elevator the Elevator subsystem.
     * @param targetPosition the desired position (in the same units as your encoder)
     * @param tolerance the acceptable range around the target position to consider "close enough"
     */
    public ElevatorPositionStopCommand( Elevator elevator, double targetPosition, double tolerance )
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
