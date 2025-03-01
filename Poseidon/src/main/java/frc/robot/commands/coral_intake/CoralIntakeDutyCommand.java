package frc.robot.commands.coral_intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class CoralIntakeDutyCommand extends Command
{
    private final CoralIntake    m_intake;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new CoralIntakeDutyCommand.
     *
     * @param intake the CoralIntake subsystem.
     * @param speedSupplier a supplier that provides a duty cycle value (between -1.0 and 1.0) to apply.
     */
    public CoralIntakeDutyCommand( CoralIntake intake, DoubleSupplier speedSupplier )
    {
        m_intake        = intake;
        m_speedSupplier = speedSupplier;

        addRequirements( m_intake );
    }

    @Override
    public void execute()
    {
        // Retrieve the duty cycle from the supplier and command the intake.
        double dutyCycle = m_speedSupplier.getAsDouble();
        m_intake.setSpeed( dutyCycle );
    }

    @Override
    public void end( boolean interrupted )
    {
        m_intake.setSpeed( 0.0 );
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
