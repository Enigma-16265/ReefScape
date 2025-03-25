package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorPositionHoldCommand extends Command {
    private final Elevator elevator;
    private final double targetPosition;
    private final double tolerance;
    private final double holdSpeed;

    /**
     * Creates a new command that drives the elevator to a target position 
     * via PID and, once within tolerance, continuously applies a hold speed until interrupted.
     *
     * @param elevator the Elevator subsystem.
     * @param targetPosition the desired position (in the same units as your encoder)
     * @param tolerance the acceptable range around the target position to consider "close enough"
     * @param holdSpeed the duty cycle speed to hold the position once within tolerance.
     */
    public ElevatorPositionHoldCommand(Elevator elevator, double targetPosition, double tolerance, double holdSpeed) {
        this.elevator = elevator;
        this.targetPosition = targetPosition;
        this.tolerance = tolerance;
        this.holdSpeed = holdSpeed;
        addRequirements(elevator);
    }

    @Override
    public void initialize() {
        // Command the elevator to drive to the target position via PID.
        elevator.setPosition(targetPosition);
    }

    @Override
    public void execute() {
        double error = Math.abs(elevator.getPosition() - targetPosition);
        // Once the error is within tolerance, apply the hold speed.
        if (error <= tolerance) {
            elevator.setSpeed(holdSpeed);
        }
        // Otherwise, do nothing; let the PID maintain the target.
    }

    @Override
    public boolean isFinished() {
        // This command is designed to never finish on its own.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the elevator when the command is interrupted.
        elevator.setSpeed(0.0);
    }
}
