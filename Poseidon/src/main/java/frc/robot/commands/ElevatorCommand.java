package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator m_elevator;
    private final DoubleSupplier m_positionSupplier;

    /**
     * Creates a new ElevatorCommand.
     *
     * @param elevator the Elevator subsystem.
     * @param positionSupplier a supplier that provides the desired elevator position (in rotations)
     */
    public ElevatorCommand(Elevator elevator, DoubleSupplier positionSupplier) {
        m_elevator = elevator;
        m_positionSupplier = positionSupplier;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        // Get the desired target position from the supplier.
        double targetPosition = m_positionSupplier.getAsDouble();
        // Command the elevator subsystem to move to the target position.
        m_elevator.setElevatorPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator by setting open-loop speed to zero.
        m_elevator.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted.
    }
}
