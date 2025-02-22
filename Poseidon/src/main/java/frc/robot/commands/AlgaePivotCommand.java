package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

public class AlgaePivotCommand extends Command {
    private final AlgaePivot m_pivot;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new RunAlgaePivot command.
     * @param pivot the AlgaePivot subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public AlgaePivotCommand(AlgaePivot pivot, DoubleSupplier speedSupplier) {
        m_pivot = pivot;
        m_speedSupplier = speedSupplier;
        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier.
        double speed = m_speedSupplier.getAsDouble();
        // Command the pivot motor with the supplied speed.
        m_pivot.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot when the command ends or is interrupted.
        m_pivot.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted.
    }
}
