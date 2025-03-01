package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import java.util.function.DoubleSupplier;

public class ElevatorHoldCommand extends Command {
    private final Elevator m_elevator;
    private final DoubleSupplier m_speedSupplier;
    private double m_holdPosition; // The last known position to hold
    private static final double DEADBAND = 0.1; // Adjust as needed

    /**
     * Creates a new ElevatorHoldCommand.
     *
     * @param elevator the Elevator subsystem.
     * @param speedSupplier a supplier that provides a speed value (typically from the left thumbstick) between -1.0 and 1.0.
     */
    public ElevatorHoldCommand(Elevator elevator, DoubleSupplier speedSupplier) {
        m_elevator = elevator;
        m_speedSupplier = speedSupplier;
        addRequirements(m_elevator);
    }

    @Override
    public void initialize() {
        // Capture the current elevator position when the command starts.
        m_holdPosition = m_elevator.getPosition();
    }

    @Override
    public void execute() {
        double speed = m_speedSupplier.getAsDouble();
        // Use a deadband to decide between manual control and holding position.
        if (Math.abs(speed) > DEADBAND) {
            // When there is significant input, drive the elevator using open-loop control.
            m_elevator.setSpeed(speed);
            // Update the hold position as you move.
            m_holdPosition = m_elevator.getPosition();
        } else {
            // With no input, command the elevator to hold its last position using closed-loop control.
            m_elevator.setPosition(m_holdPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // When the command ends, stop the elevator.
        m_elevator.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously as the default.
    }
}
