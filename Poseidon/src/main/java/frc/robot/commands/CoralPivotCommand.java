package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

public class CoralPivotCommand extends Command {
    private final CoralPivot m_pivot;
    private final DoubleSupplier m_positionSupplier;

    /**
     * Creates a new RunCoralPivot command.
     * 
     * @param pivot the CoralPivot subsystem.
     * @param positionSupplier a supplier that provides a pivot position (e.g., between -1.0 and 1.0)
     */
    public CoralPivotCommand(CoralPivot pivot, DoubleSupplier positionSupplier) {
        m_pivot = pivot;
        m_positionSupplier = positionSupplier;
        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        // Retrieve the desired pivot position from the supplier.
        double targetPosition = m_positionSupplier.getAsDouble();
        // Command the CoralPivot subsystem to move to the target position using closed-loop control.
        m_pivot.setPivotPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop or hold the pivot when the command ends.
        m_pivot.setPivotPosition(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}
