package frc.robot.commands.algae_intake;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeIntakeDutyCommand extends Command {
    private final AlgaeIntake m_intake;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new AlgaeIntakeDutyCommand.
     *
     * @param intake the AlgaeIntake subsystem.
     * @param speedSupplier a supplier that provides a duty cycle value between -1.0 and 1.0.
     */
    public AlgaeIntakeDutyCommand(AlgaeIntake intake, DoubleSupplier speedSupplier) {
        m_intake = intake;
        m_speedSupplier = speedSupplier;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        // Retrieve the duty cycle from the supplier (expected to be between -1.0 and 1.0)
        double dutyCycle = m_speedSupplier.getAsDouble();
        // Command the intake with the provided duty cycle in open-loop control.
        m_intake.setSpeed(dutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake when the command ends or is interrupted.
        m_intake.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}
