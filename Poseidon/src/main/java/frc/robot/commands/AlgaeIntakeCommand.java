package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;

public class AlgaeIntakeCommand extends Command {
    private final AlgaeIntake m_intake;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new RunAlgaeIntake command.
     *
     * @param intake the AlgaeIntake subsystem.
     * @param speedSupplier a supplier that provides a speed value between 0.0 and 1.0.
     */
    public AlgaeIntakeCommand(AlgaeIntake intake, DoubleSupplier speedSupplier) {
        m_intake = intake;
        m_speedSupplier = speedSupplier;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier (expected to be between 0.0 and 1.0).
        double speedValue = m_speedSupplier.getAsDouble();
        // Map the speed value to a target RPM. For example, a value of 1.0 will command kNoLoadRpm.
        double targetRPM = speedValue * AlgaeIntake.kNoLoadRpm;
        // Command the intake to run at the target RPM using closed-loop control.
        m_intake.setIntakeSpeed(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake when the command ends or is interrupted.
        m_intake.setIntakeSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}