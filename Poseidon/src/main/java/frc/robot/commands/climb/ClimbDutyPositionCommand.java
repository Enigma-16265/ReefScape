package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Climb;

public class ClimbDutyPositionCommand extends Command {
    private final Climb climb;
    private final double targetPosition;
    private final double speed;      // Speed magnitude (absolute value) to drive the motor
    private final double tolerance;  // Acceptable error from the target position

    public ClimbDutyPositionCommand( Climb climb, double targetPosition, double speed, double tolerance )
    {
        this.climb          = climb;
        this.targetPosition = targetPosition;
        this.speed          = speed;
        this.tolerance      = tolerance;

        addRequirements( climb );
    }

    @Override
    public void initialize()
    {
        // Optionally, perform any necessary initialization.
    }

    @Override
    public void execute()
    {
        double currentPos = climb.getPosition();
        double error = targetPosition - currentPos;

        // If the error is larger than the tolerance, drive with the provided speed in the correct direction.
        double commandedSpeed = (Math.abs(error) > tolerance) ? Math.copySign(speed, error) : 0.0;

        // Call setSpeed only once per execution cycle.
        climb.setSpeed(commandedSpeed);
    }

    @Override
    public boolean isFinished()
    {
        return Math.abs( climb.getPosition() - targetPosition ) <= tolerance;
    }

    @Override
    public void end( boolean interrupted )
    {
        // Stop the motor when the command ends.
        climb.setSpeed( 0.0 );
    }
}

