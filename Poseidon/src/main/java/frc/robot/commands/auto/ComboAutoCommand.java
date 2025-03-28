package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swervedrive.DriveForwardTimedCommand;
import frc.robot.commands.algae_pivot.AlgaePivotPositionCommand;
import frc.robot.commands.algae_pivot.AlgaePivotPositionStopCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.AlgaePivot;

public class ComboAutoCommand extends SequentialCommandGroup {

    /**
     * Static factory method that creates a new ComboAutoCommand instance.
     * 
     * @param drivebase  the swerve drive subsystem.
     * @param algaePivot the AlgaePivot subsystem.
     * @return a new ComboAutoCommand instance.
     */
    public static ComboAutoCommand getInstance(SwerveSubsystem drivebase, AlgaePivot algaePivot) {
        return new ComboAutoCommand(drivebase, algaePivot);
    }

    /**
     * Private constructor to build the command sequence.
     * 
     * The sequence is:
     * 1. Drive forward at -1.0 m/s for 3.5 seconds.
     * 2. Instantly command the algae pivot to 25° (the PID remains active).
     * 3. Wait 1 second.
     * 4. Stop the algae pivot by returning it to near zero and stop the pid.
     *
     * @param drivebase  the swerve drive subsystem.
     * @param algaePivot the AlgaePivot subsystem.
     */
    private ComboAutoCommand(SwerveSubsystem drivebase, AlgaePivot algaePivot) {
        addCommands(
            // Step 1: Drive forward for 3.5 seconds.
            new DriveForwardTimedCommand(drivebase, -1.0, 3.5),
            // Step 2: Instantly set the algae pivot to 25° via its PID.
            new AlgaePivotPositionCommand(algaePivot, 25.0),
            // Step 3: Wait 1 second while the PID remains active.
            new WaitCommand(1.0),
            // Step 4: Stop the algae pivot (disables the PID output).
            new AlgaePivotPositionStopCommand(algaePivot, 0.0, 5.0)
        );
    }
}
