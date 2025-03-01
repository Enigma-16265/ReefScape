package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;
import java.util.function.BooleanSupplier;

public class ClimbHoldCommand extends Command
{
    private final Climb           m_climb;
    private final BooleanSupplier m_upButton;
    private final BooleanSupplier m_downButton;
    private final double          m_upSpeed;
    private final double          m_downSpeed;
    private double                m_holdPosition; // The last recorded position to hold

    /**
     * Creates a new ClimbHoldCommand.
     *
     * @param climb the Climb subsystem.
     * @param upButton a BooleanSupplier that returns true when the "up" (climb) button is pressed.
     * @param downButton a BooleanSupplier that returns true when the "down" (let down) button is pressed.
     * @param upSpeed the speed at which to climb (positive value, e.g. 0.5).
     * @param downSpeed the speed at which to lower (negative value, e.g. -0.5).
     */
    public ClimbHoldCommand( Climb climb, BooleanSupplier upButton, BooleanSupplier downButton, double upSpeed, double downSpeed )
    {
        m_climb      = climb;
        m_upButton   = upButton;
        m_downButton = downButton;
        m_upSpeed    = upSpeed;
        m_downSpeed  = downSpeed;

        addRequirements( m_climb );
    }

    @Override
    public void initialize()
    {
        // Capture the current climb position when the command starts.
        m_holdPosition = m_climb.getPosition();
    }

    @Override
    public void execute()
    {
        double speed = 0.0;
        if ( m_upButton.getAsBoolean() && !m_downButton.getAsBoolean() )
        {
            speed = m_upSpeed;
        } 
        else if ( m_downButton.getAsBoolean() && !m_upButton.getAsBoolean() )
        {
            speed = m_downSpeed;
        } 
        else
        {
            speed = 0.0;
        }
        
        if ( Math.abs( speed ) > 0.0 )
        {
            // With active input, drive the climb in open-loop and update the hold position.
            m_climb.setSpeed( speed );
            m_holdPosition = m_climb.getPosition();
        }
        else
        {
            // With no button pressed, hold the last recorded position.
            m_climb.setPosition( m_holdPosition );
        }
    }

    @Override
    public void end( boolean interrupted )
    {
        m_climb.setSpeed( 0.0 );
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}
