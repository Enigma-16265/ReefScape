package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommand extends Command {
    private final Elevator m_elevator;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new ElevatorCommand.
     * 
     * @param elevator the Elevator subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public ElevatorCommand(Elevator elevator, DoubleSupplier speedSupplier) {
        m_elevator = elevator;
        m_speedSupplier = speedSupplier;
        addRequirements(m_elevator);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier.
        double speed = m_speedSupplier.getAsDouble();
        // Command the elevator motor with the supplied speed.
        m_elevator.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator when the command ends or is interrupted.
        m_elevator.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted.
    }
}
