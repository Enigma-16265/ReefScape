package frc.robot.commands.coral_pivot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralPivot;
import java.util.function.DoubleSupplier;

public class CoralPivotHoldCommand extends Command
{
    private final CoralPivot m_pivot;
    private final DoubleSupplier m_ySupplier;
    private double m_holdPosition;
    private static final double DEADBAND = 0.1; // Adjust as needed

    /**
     * Creates a new CoralPivotHoldCommand.
     *
     * @param pivot the CoralPivot subsystem.
     * @param ySupplier a supplier that provides the right thumbstick Y value (expected between -1.0 and 1.0).
     */
    public CoralPivotHoldCommand(CoralPivot pivot, DoubleSupplier ySupplier) {
        m_pivot = pivot;
        m_ySupplier = ySupplier;
        addRequirements(m_pivot);
    }

    @Override
    public void initialize() {
        // Capture the current position when starting.
        m_holdPosition = m_pivot.getPosition();
    }

    @Override
    public void execute() {
        double speed = m_ySupplier.getAsDouble();
        // If the thumbstick is moved beyond the deadband, drive open-loop and update hold position.
        if (Math.abs(speed) > DEADBAND) {
            m_pivot.setSpeed(speed);
            m_holdPosition = m_pivot.getPosition();
        } else {
            // If the thumbstick is neutral, hold the last recorded position using closed-loop control.
            m_pivot.setPosition(m_holdPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot when the command ends.
        m_pivot.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously.
    }
}
