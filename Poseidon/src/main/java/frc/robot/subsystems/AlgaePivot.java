package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.epilogue.Logged;
import frc.logging.DataNetworkTableLog;

//@Logged
public class AlgaePivot extends SubsystemBase
{
    private static final DataNetworkTableLog tlmLog =
        new DataNetworkTableLog( 
            "Subsystems.AlgaePivot.tlm",
            Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "current", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final DataNetworkTableLog cmdLog =
    new DataNetworkTableLog( 
        "Subsystems.AlgaePivot.cmd",
        Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    public static final int    kPivotMotorCanId  = 5;
    public static final double kPivotGearRatio   = 1.0 / 45.0;
    public static final double kNoLoadRpm        = 5500 * kPivotGearRatio;
    public static final double kMinRotPos        = 0.0;
    public static final double kMaxRotPos        = 9.0;
    public static final double kCurrentThreshold = 20.0;

    // PID tuning parameters for position control (to be tuned)
    private static final double kP_pos = 0.1;
    private static final double kI_pos = 0.0;
    private static final double kD_pos = 0.0;

    // Flags to enable/disable safety checks
    private boolean encoderCheckEnabled = false;
    private boolean currentCheckEnabled = false;

    private final SparkMax m_pivotSparkMax;
    public final SparkMaxConfig m_pivotSparkMaxConfig;

    private final RelativeEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotClosedLoopController;

    public AlgaePivot()
    {
        m_pivotSparkMaxConfig = new SparkMaxConfig();
        // Configure conversion factors: position conversion factor accounts for gear reduction.
        m_pivotSparkMaxConfig.encoder.positionConversionFactor(kPivotGearRatio);
        // Velocity conversion: raw rotations per second * 60 = RPM, then account for gear reduction.
        m_pivotSparkMaxConfig.encoder.velocityConversionFactor(kPivotGearRatio * 60.0);

        // Configure PID parameters and output limits.`
        m_pivotSparkMaxConfig.closedLoop.pid(kP_pos, kI_pos, kD_pos);
        m_pivotSparkMaxConfig.closedLoop.outputRange(-1.0, 1.0);
        m_pivotSparkMaxConfig.closedLoop.maxMotion.maxAcceleration(400.0);
        m_pivotSparkMaxConfig.closedLoop.maxMotion.maxVelocity(80.0);

        m_pivotSparkMax = new SparkMax(kPivotMotorCanId, MotorType.kBrushless);
        m_pivotSparkMax.configure(m_pivotSparkMaxConfig, null, null);

        m_pivotEncoder = m_pivotSparkMax.getEncoder();
        m_pivotEncoder.setPosition(0.0);

        m_pivotClosedLoopController = m_pivotSparkMax.getClosedLoopController();
    }

    /**
     * Enables or disables the encoder safety check.
     *
     * @param enabled true to enable, false to disable.
     */
    public void setEncoderCheckEnabled(boolean enabled) {
        encoderCheckEnabled = enabled;
    }

    /**
     * Enables or disables the current draw safety check.
     *
     * @param enabled true to enable, false to disable.
     */
    public void setCurrentCheckEnabled(boolean enabled) {
        currentCheckEnabled = enabled;
    }

    /**
     * Directly sets the motor output while enforcing a current draw safety check.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed( double speed )
    {
        if ( currentCheckEnabled )
        {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if ( currentDraw > kCurrentThreshold )
            {
                m_pivotSparkMax.set( 0.0 );
                return;
            }
        }

        cmdLog.publish( "speed", speed );

        m_pivotSparkMax.set( speed );

    }

    /**
     * Sets the pivot position using the PID controller while enforcing safety limits.
     * This method first applies the safety checks, then clamps the final value
     * between kMinRotPos and kMaxRotPos just before commanding the motor.
     *
     * @param position the desired pivot position (in rotations)
     */
    public void setPosition(double position)
    {
        double targetPosition = position;
        
        if (encoderCheckEnabled)
        {
            double currentPosition = m_pivotEncoder.getPosition();
            // Prevent driving further past the physical limits.
            if (targetPosition > currentPosition && currentPosition >= kMaxRotPos)
            {
                targetPosition = currentPosition;
            }
            else if (targetPosition < currentPosition && currentPosition <= kMinRotPos)
            {
                targetPosition = currentPosition;
            }
        }
    
        if (currentCheckEnabled)
        {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                // Hold the current position when current is too high.
                targetPosition = m_pivotEncoder.getPosition();
            }
        }
    
        // Clamp the target position just before commanding the motor.
        targetPosition = MathUtil.clamp(targetPosition, kMinRotPos, kMaxRotPos);

        cmdLog.publish( "position", position );

        m_pivotClosedLoopController.setReference(targetPosition, ControlType.kPosition);

    }

    /**
     * Sets the pivot velocity using the PID controller while enforcing current draw safety.
     * If the current draw exceeds the threshold, the motor is commanded to 0 velocity.
     * The final velocity is clamped between -kNoLoadRpm and kNoLoadRpm just before command.
     *
     * @param velocity the desired velocity (in RPM)
     */
    public void setVelocity(double velocity)
    {
        double targetVelocity = velocity;
        
        if (currentCheckEnabled)
        {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                targetVelocity = 0.0;
            }
        }
        
        // Clamp the final velocity just before commanding the motor.
        targetVelocity = MathUtil.clamp(targetVelocity, -kNoLoadRpm, kNoLoadRpm);

        cmdLog.publish( "velocity", velocity );

        m_pivotClosedLoopController.setReference(targetVelocity, ControlType.kVelocity);

    }

    public double getVelocity()
    {
        return m_pivotEncoder.getVelocity();
    }

    public double getPosition()
    {
        return m_pivotEncoder.getPosition();
    }

    public double getCurrent()
    {
        return m_pivotSparkMax.getOutputCurrent();
    }

    public void logValues()
    {
        tlmLog.publish( "velocity", m_pivotEncoder.getVelocity() );
        tlmLog.publish( "position", m_pivotEncoder.getPosition() );
        tlmLog.publish( "current",  m_pivotSparkMax.getOutputCurrent() );
    }
    
}
