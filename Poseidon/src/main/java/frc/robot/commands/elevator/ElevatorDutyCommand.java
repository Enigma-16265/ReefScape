package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Elevator;

public class ElevatorDutyCommand extends Command
{
    // Compile-time constants for the scaling factor bounds.
    public static final double MAX_SCALING = 1.0;
    public static final double MIN_SCALING = -1.0;

    private final Elevator       m_elevator;
    private final DoubleSupplier m_speedSupplier;
    private final double         m_scalingFactor;

    /**
     * Creates a new ElevatorDutyCommand with a default scaling factor of 1.0.
     *
     * @param elevator the Elevator subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public ElevatorDutyCommand( Elevator elevator, DoubleSupplier speedSupplier )
    {
        this( elevator, speedSupplier, 1.0 );
    }

    /**
     * Creates a new ElevatorDutyCommand with an optional scaling factor.
     * The provided scaling factor is clamped to be between MIN_SCALING and MAX_SCALING.
     *
     * @param elevator the Elevator subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     * @param scalingFactor a scaling factor to modify the supplied speed.
     */
    public ElevatorDutyCommand( Elevator elevator, DoubleSupplier speedSupplier, double scalingFactor )
    {
        m_elevator       = elevator;
        m_speedSupplier  = speedSupplier;
        m_scalingFactor  = MathUtil.clamp( scalingFactor, MIN_SCALING, MAX_SCALING );
        
        addRequirements( m_elevator );
    }

    @Override
    public void execute()
    {
        double speed       = m_speedSupplier.getAsDouble();
        double scaledSpeed = speed * m_scalingFactor;
        
        m_elevator.setSpeed( scaledSpeed );
    }

    @Override
    public void end( boolean interrupted )
    {
        m_elevator.setSpeed( 0.0 );
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
