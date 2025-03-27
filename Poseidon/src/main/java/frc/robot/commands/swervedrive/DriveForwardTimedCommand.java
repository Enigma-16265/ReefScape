package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

public class DriveForwardTimedCommand extends Command {
    private final SwerveSubsystem drivebase;
    private final Timer timer = new Timer();
    private final double duration;   // Duration in seconds
    private final double velocity;   // Forward velocity in m/s

    /**
     * Creates a command that drives the robot forward at a given velocity for a specified duration.
     *
     * @param drivebase the swerve drive subsystem.
     * @param velocity  the forward velocity in meters per second (positive for forward, negative for backward)
     * @param duration  the time in seconds to drive at that velocity.
     */
    public DriveForwardTimedCommand(SwerveSubsystem drivebase, double velocity, double duration) {
        this.drivebase = drivebase;
        this.velocity = velocity;
        this.duration = duration;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        // Drive the robot forward at the specified velocity (field-relative) with zero rotation.
        drivebase.drive(new Translation2d(velocity, 0.0), 0.0, true);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(duration);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the drive by commanding zero translation and zero rotation.
        drivebase.drive(new Translation2d(0.0, 0.0), 0.0, true);
        timer.stop();
    }
}
