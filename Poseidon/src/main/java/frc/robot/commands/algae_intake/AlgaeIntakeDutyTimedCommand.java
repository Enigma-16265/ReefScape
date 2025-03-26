package frc.robot.commands.algae_intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeIntake;
import edu.wpi.first.wpilibj.Timer;

public class AlgaeIntakeDutyTimedCommand extends Command {
    private final AlgaeIntake m_intake;
    private final double m_speed;
    private final double m_duration;
    private final Timer m_timer = new Timer();

    /**
     * Creates a new AlgaeIntakeDutyTimedCommand.
     *
     * @param intake the AlgaeIntake subsystem.
     * @param speed the duty cycle speed to apply (from -1.0 to 1.0)
     * @param duration the time in seconds to run the intake at the specified speed.
     */
    public AlgaeIntakeDutyTimedCommand(AlgaeIntake intake, double speed, double duration) {
        m_intake = intake;
        m_speed = speed;
        m_duration = duration;
        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_timer.reset();
        m_timer.start();
    }

    @Override
    public void execute() {
        m_intake.setSpeed(m_speed);
    }

    @Override
    public boolean isFinished() {
        return m_timer.get() >= m_duration;
    }

    @Override
    public void end(boolean interrupted) {
        m_intake.setSpeed(0.0);
        m_timer.stop();
    }
}
