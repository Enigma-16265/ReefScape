package frc.robot.commands.algae_pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaePivot;

/**
 * An InstantCommand that sets a target pivot position.
 * When executed, it commands the AlgaePivot subsystem to move to (and hold) the desired position.
 */
public class AlgaePivotPositionCommand extends InstantCommand {

    /**
     * Creates a new AlgaePivotPositionCommand.
     *
     * @param pivot the AlgaePivot subsystem.
     * @param targetPosition the desired pivot position (in revolutions).
     */
    public AlgaePivotPositionCommand(AlgaePivot pivot, double targetPosition) {
        super(() -> pivot.setPosition(targetPosition), pivot);
    }
}
