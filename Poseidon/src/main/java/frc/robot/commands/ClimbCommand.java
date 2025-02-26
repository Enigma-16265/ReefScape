package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbCommand extends Command {
    private final Climb m_climb;
    private final DoubleSupplier m_speedSupplier;

    /**
     * Creates a new RunClimb command.
     *
     * @param climb the Climb subsystem.
     * @param speedSupplier a supplier that provides a speed value (-1.0 to 1.0)
     */
    public ClimbCommand(Climb climb, DoubleSupplier speedSupplier) {
        m_climb = climb;
        m_speedSupplier = speedSupplier;
        addRequirements(m_climb);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier and command the climb subsystem.
        double speed = m_speedSupplier.getAsDouble();
        m_climb.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the climb when the command ends or is interrupted.
        m_climb.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // Runs continuously until interrupted.
    }
}
