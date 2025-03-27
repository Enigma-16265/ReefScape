package frc.robot.commands.swervedrive;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class DriveForwardTimedCommand {
    /**
     * Creates a command that drives the robot forward at 1 m/s for the specified duration.
     *
     * @param drivebase the YAGSL-based swerve drive subsystem.
     * @param duration the duration (in seconds) to drive forward.
     * @return a command that drives forward for the specified time and then stops.
     */
    public static Command create(SwerveSubsystem drivebase, double duration) {
        // Create a SwerveInputStream that commands a constant forward speed.
        SwerveInputStream constantForward = SwerveInputStream.of(
            drivebase.getSwerveDrive(),
            () -> -0.1,   // Forward speed in m/s (adjust sign as needed)
            () -> 0.0     // Lateral speed in m/s
        ).withControllerRotationAxis(() -> 0.0);  // Zero rotational speed

        // Command that drives with the constant input.
        Command driveForward = drivebase.driveFieldOriented(constantForward);

        // Create a command that stops the drive by commanding zeros.
        Command stopDrive = new InstantCommand(() -> {
            SwerveInputStream zeroStream = SwerveInputStream.of(
                drivebase.getSwerveDrive(),
                () -> 0.0,
                () -> 0.0
            ).withControllerRotationAxis(() -> 0.0);
            drivebase.driveFieldOriented(zeroStream);
        }, drivebase);

        // Create a command to disable the default command.
        Command disableDefault = new InstantCommand(() -> drivebase.setDefaultCommand(null), drivebase);
        // (Optionally, create a command to re-enable the default command if needed.)
        // For example, if you have a variable `defaultDriveCommand`, you can restore it:
        // Command enableDefault = new InstantCommand(() -> drivebase.setDefaultCommand(defaultDriveCommand), drivebase);

        // Sequence the commands:
        // 1. Disable the default command.
        // 2. Run the driveForward command for the duration.
        // 3. Stop the drive.
        // 4. (Optionally) Re-enable the default command.
        return Commands.sequence(
            disableDefault,
            Commands.deadline(driveForward, new WaitCommand(duration)),
            stopDrive
            // , enableDefault   // Uncomment if you need to restore the default command.
        );
    }
}
