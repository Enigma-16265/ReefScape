package frc.robot.commands.climb;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Climb;

public class ClimbDutyCommand extends Command
{
    // Compile-time constants for the scaling factor bounds.
    public static final double MAX_SCALING = 1.0;
    public static final double MIN_SCALING = -1.0;

    private final Climb          m_climb;
    private final DoubleSupplier m_speedSupplier;
    private final double         m_scalingFactor;

    /**
     * Creates a new ClimbDutyCommand with a default scaling factor of 1.0.
     * @param climb the Climb subsystem.
     * @param speedSupplier a supplier that provides a speed value (-1.0 to 1.0).
     */
    public ClimbDutyCommand( Climb climb, DoubleSupplier speedSupplier )
    {
        this( climb, speedSupplier, 1.0 );
    }

    /**
     * Creates a new ClimbDutyCommand with an optional scaling factor.
     * The provided scaling factor is clamped to be between MIN_SCALING and MAX_SCALING.
     * @param climb the Climb subsystem.
     * @param speedSupplier a supplier that provides a speed value (-1.0 to 1.0).
     * @param scalingFactor a scaling factor to modify the supplied speed.
     */
    public ClimbDutyCommand( Climb climb, DoubleSupplier speedSupplier, double scalingFactor )
    {
        m_climb            = climb;
        m_speedSupplier    = speedSupplier;
        m_scalingFactor    = MathUtil.clamp( scalingFactor, MIN_SCALING, MAX_SCALING );

        addRequirements( m_climb );
    }

    @Override
    public void execute()
    {
        double speed       = m_speedSupplier.getAsDouble();
        double scaledSpeed = speed * m_scalingFactor;
        
        m_climb.setSpeed( scaledSpeed );
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
