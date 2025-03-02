package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;

/**
 * An InstantCommand that sets a target elevator position.
 * When executed, it commands the Elevator subsystem to move to (and hold) the desired position.
 */
public class ElevatorPositionCommand extends InstantCommand
{
    /**
     * Creates a new ElevatorPositionCommand.
     * 
     * @param elevator the Elevator subsystem.
     * @param targetPosition the desired elevator position (in appropriate units, e.g. rotations or inches).
     */
    public ElevatorPositionCommand( Elevator elevator, double targetPosition )
    {
        super( () -> elevator.setPosition( targetPosition ), elevator );
    }
}
