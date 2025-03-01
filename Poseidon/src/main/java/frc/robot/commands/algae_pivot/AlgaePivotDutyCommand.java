package frc.robot.commands.algae_pivot;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaePivot;

public class AlgaePivotDutyCommand extends Command {
    private final AlgaePivot m_pivot;
    private final DoubleSupplier m_speedSupplier;
    private final double m_scalingFactor;

    /**
     * Creates a new RunAlgaePivot command.
     * @param pivot the AlgaePivot subsystem.
     * @param speedSupplier a supplier that provides a speed value between -1.0 and 1.0.
     */
    public AlgaePivotDutyCommand(AlgaePivot pivot, DoubleSupplier speedSupplier, double scalingFactor ) {
        m_pivot = pivot;
        m_speedSupplier = speedSupplier;

        if ( scalingFactor > 1.0 )
        {
            scalingFactor = 1.0;
        } else if ( scalingFactor < -1.0 )
        {
            scalingFactor = -1.0;
        }

        m_scalingFactor = scalingFactor;

        addRequirements(m_pivot);
    }

    @Override
    public void execute() {
        // Retrieve the speed value from the supplier.
        double dutyCycle       = m_speedSupplier.getAsDouble();
        double scaledDutyCycle = dutyCycle * m_scalingFactor;
        // Command the pivot motor with the supplied speed.
        m_pivot.setSpeed( scaledDutyCycle );
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot when the command ends or is interrupted.
        m_pivot.setSpeed(0.0);
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted.
    }
}
