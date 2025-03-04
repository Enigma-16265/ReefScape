package frc.robot.commands.coral_pivot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralPivot;

public class CoralPivotPositionCommand extends InstantCommand
{

    /**
     * Creates a new CoralPivotPositionCommand.
     *
     * @param pivot the CoralPivot subsystem.
     * @param targetPosition the desired pivot position (in degrees).
     */
    public CoralPivotPositionCommand( CoralPivot pivot, double targetPosition )
    {
        super( () -> pivot.setPosition( targetPosition ), pivot );
    }

}
