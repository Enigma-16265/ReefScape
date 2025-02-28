package frc.robot.commands.coral_intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntake;
import java.util.function.BooleanSupplier;

public class CoralIntakeHoldCommand extends Command
{
    private final CoralIntake m_intake;
    private final BooleanSupplier m_intakeButton;   // e.g., X button
    private final BooleanSupplier m_outtakeButton;  // e.g., A button
    private final double m_intakeSpeed;
    private final double m_outtakeSpeed;

    /**
     * Creates a new CoralIntakeHoldCommand.
     *
     * @param intake the CoralIntake subsystem.
     * @param intakeButton a BooleanSupplier that returns true when the intake (e.g., X) button is pressed.
     * @param outtakeButton a BooleanSupplier that returns true when the outtake (e.g., A) button is pressed.
     * @param intakeSpeed the constant speed (positive value) to run for intake.
     * @param outtakeSpeed the constant speed (negative value) to run for outtake.
     */
    public CoralIntakeHoldCommand(CoralIntake intake,
                                  BooleanSupplier intakeButton,
                                  BooleanSupplier outtakeButton,
                                  double intakeSpeed,
                                  double outtakeSpeed) {
        m_intake = intake;
        m_intakeButton = intakeButton;
        m_outtakeButton = outtakeButton;
        m_intakeSpeed = intakeSpeed;
        m_outtakeSpeed = outtakeSpeed;
        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (m_intakeButton.getAsBoolean() && !m_outtakeButton.getAsBoolean()) {
            // Intake: set the motor to run at the specified intake speed.
            m_intake.setSpeed(m_intakeSpeed);
        } else if (m_outtakeButton.getAsBoolean() && !m_intakeButton.getAsBoolean()) {
            // Outtake: set the motor to run at the specified outtake speed.
            m_intake.setSpeed(m_outtakeSpeed);
        } else {
            // No buttons pressed (or both pressed): stop the motor.
            m_intake.setSpeed(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the intake when the command ends or is interrupted.
        m_intake.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs continuously until interrupted.
    }
}
