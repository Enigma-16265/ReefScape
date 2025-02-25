package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;

public class CoralPivotCommand extends Command {
    private final CoralPivot m_pivot;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new CoralPivotCommand.
     * 
     * @param pivot the CoralPivot subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public CoralPivotCommand(CoralPivot pivot, DoubleSupplier speedSupplier) {
        m_pivot = pivot;
        m_speedSupplier = speedSupplier;
        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        // Retrieve the desired speed value from the supplier.
        double targetSpeed = m_speedSupplier.getAsDouble();
        // Command the CoralPivot subsystem to run at the target speed.
        m_pivot.setSpeed(targetSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot when the command ends or is interrupted.
        m_pivot.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}
