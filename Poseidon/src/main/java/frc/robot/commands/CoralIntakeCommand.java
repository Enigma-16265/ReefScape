package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;

public class CoralIntakeCommand extends Command {
    private final CoralIntake m_intake;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new RunCoralIntake command.
     * @param intake the CoralIntake subsystem.
     * @param speedSupplier a supplier that provides a speed value (0.0 to 1.0)
     */
    public CoralIntakeCommand(CoralIntake intake, DoubleSupplier speedSupplier) {
        m_intake = intake;
        m_speedSupplier = speedSupplier;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        // Retrieve the speed value (expected between 0.0 and 1.0).
        double speedValue = m_speedSupplier.getAsDouble();
        // Map the speed value to target RPM using the no-load RPM defined in CoralIntake.
        double targetRPM = speedValue * CoralIntake.kNoLoadRpm;
        m_intake.setVelocity(targetRPM);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake when the command ends or is interrupted.
        m_intake.setVelocity(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}
