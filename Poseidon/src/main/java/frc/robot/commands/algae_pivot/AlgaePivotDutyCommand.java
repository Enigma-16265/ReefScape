package frc.robot.commands.algae_pivot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.AlgaePivot;

public class AlgaePivotDutyCommand extends Command
{
    // Compile-time constants for the scaling factor bounds.
    public static final double MAX_SCALING = 1.0;
    public static final double MIN_SCALING = -1.0;

    private final AlgaePivot     m_pivot;
    private final DoubleSupplier m_speedSupplier;
    private final double         m_scalingFactor;

    /**
     * Creates a new AlgaePivotDutyCommand with a default scaling factor of 1.0.
     * @param pivot the AlgaePivot subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public AlgaePivotDutyCommand( AlgaePivot pivot, DoubleSupplier speedSupplier )
    {
        this(pivot, speedSupplier, 1.0 );
    }

    /**
     * Creates a new AlgaePivotDutyCommand with an optional scaling factor.
     * The provided scaling factor is clamped to be between MIN_SCALING and MAX_SCALING.
     * @param pivot the AlgaePivot subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     * @param scalingFactor a scaling factor to modify the supplied speed.
     */
    public AlgaePivotDutyCommand( AlgaePivot pivot, DoubleSupplier speedSupplier, double scalingFactor )
    {
        m_pivot         = pivot;
        m_speedSupplier = speedSupplier;
        m_scalingFactor = MathUtil.clamp( scalingFactor, MIN_SCALING, MAX_SCALING );
        
        addRequirements( m_pivot );
    }

    @Override
    public void execute()
    {
        double speed       = m_speedSupplier.getAsDouble();
        double scaledSpeed = speed * m_scalingFactor;

        m_pivot.setSpeed( scaledSpeed );
    }

    @Override
    public void end( boolean interrupted )
    {
        m_pivot.setSpeed( 0.0 );
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

}
