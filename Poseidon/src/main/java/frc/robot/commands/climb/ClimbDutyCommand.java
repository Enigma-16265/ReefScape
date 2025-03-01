package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbDutyCommand extends Command {
    private final Climb m_climb;
    private final DoubleSupplier m_speedSupplier;
    private final double m_scalingFactor;

    /**
     * Creates a new RunClimb command.
     *
     * @param climb the Climb subsystem.
     * @param speedSupplier a supplier that provides a speed value (-1.0 to 1.0)
     */
    public ClimbDutyCommand(Climb climb, DoubleSupplier speedSupplier, double scalingFactor ) {
        m_climb = climb;
        m_speedSupplier = speedSupplier;

        if ( scalingFactor > 1.0 )
        {
            scalingFactor = 1.0;
        } else if ( scalingFactor < -1.0 )
        {
            scalingFactor = -1.0;
        }

        m_scalingFactor = scalingFactor;
        addRequirements(m_climb);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier and command the climb subsystem.
        double dutyCycle = m_speedSupplier.getAsDouble();
        double scaledDutyCycle = dutyCycle * m_scalingFactor;

        m_climb.setSpeed(scaledDutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the climb when the command ends or is interrupted.
        m_climb.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted.
    }
}
