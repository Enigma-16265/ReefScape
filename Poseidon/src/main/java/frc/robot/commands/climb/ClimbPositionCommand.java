package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climb;

public class ClimbPositionCommand extends InstantCommand
{

    public ClimbPositionCommand( Climb climb, double targetPosition )
    {
        super( () -> climb.setPosition( targetPosition ), climb );
    }

}
