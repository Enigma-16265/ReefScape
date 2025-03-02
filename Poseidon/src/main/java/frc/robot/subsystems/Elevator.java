package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.epilogue.Logged;
import frc.logging.DataNetworkTableLog;

//@Logged
public class Elevator extends SubsystemBase
{
    private static final DataNetworkTableLog tlmLog =
        new DataNetworkTableLog( 
            "Subsystems.Elevator.tlm",
            Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "current", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final DataNetworkTableLog cmdLog =
    new DataNetworkTableLog( 
        "Subsystems.Elevator.cmd",
        Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    public static final int    kElevatorMasterCanId   = 21;
    public static final int    kElevatorFollowerCanId = 22;
    public static final double kElevatorGearRatio     = 1.0 / 5.0;
    public static final double kNoLoadRpm             = 5500 * kElevatorGearRatio;
    public static final double kMinRotPos             = 0.0;
    public static final double kMaxRotPos             = 20.0;
    public static final double kCurrentThreshold      = 20.0;

    // PID tuning parameters for position control (to be tuned)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Flags to enable/disable safety checks
    private boolean encoderCheckEnabled = true;
    private boolean currentCheckEnabled = true;

    private final SparkFlex m_masterMotor;
    private final SparkFlex m_followerMotor;
    
    // Separate config objects for master and follower.
    public final SparkFlexConfig m_elevatorMasterConfig;
    public final SparkFlexConfig m_elevatorFollowerConfig;

    private final RelativeEncoder m_elevatorEncoder;
    private final SparkClosedLoopController m_elevatorClosedLoopController;

    public Elevator()
    {
        // Create and configure the master config.
        m_elevatorMasterConfig = new SparkFlexConfig();
        m_elevatorMasterConfig.encoder.positionConversionFactor(kElevatorGearRatio);
        m_elevatorMasterConfig.encoder.velocityConversionFactor(kElevatorGearRatio * 60.0);
        m_elevatorMasterConfig.closedLoop.pid(kP, kI, kD);
        m_elevatorMasterConfig.closedLoop.outputRange(-1.0, 1.0);

        // Create and configure the follower config.
        m_elevatorFollowerConfig = new SparkFlexConfig();
        // If you need the follower inverted relative to the master, set it here.
        m_elevatorFollowerConfig.inverted(true);
        m_elevatorFollowerConfig.follow( kElevatorFollowerCanId );

        m_masterMotor = new SparkFlex(kElevatorMasterCanId, MotorType.kBrushless);
        m_masterMotor.configure(m_elevatorMasterConfig, null, null);

        m_followerMotor = new SparkFlex(kElevatorFollowerCanId, MotorType.kBrushless);
        m_followerMotor.configure(m_elevatorFollowerConfig, null, null);

        m_elevatorEncoder = m_masterMotor.getEncoder();
        m_elevatorEncoder.setPosition(0.0);

        m_elevatorClosedLoopController = m_masterMotor.getClosedLoopController();
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
            double currentDraw = m_masterMotor.getOutputCurrent();
            if ( currentDraw > kCurrentThreshold )
            {
                m_masterMotor.set( 0.0 );
                return;
            }
        }

        cmdLog.publish( "speed", speed );

        m_masterMotor.set( speed );

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
            double currentPosition = m_elevatorEncoder.getPosition();
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
            double currentDraw = m_masterMotor.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                // Hold the current position when current is too high.
                targetPosition = m_elevatorEncoder.getPosition();
            }
        }
    
        // Clamp the target position just before commanding the motor.
        targetPosition = MathUtil.clamp(targetPosition, kMinRotPos, kMaxRotPos);

        cmdLog.publish( "position", position );

        m_elevatorClosedLoopController.setReference(targetPosition, ControlType.kPosition);

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
            double currentDraw = m_masterMotor.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                targetVelocity = 0.0;
            }
        }
        
        // Clamp the final velocity just before commanding the motor.
        targetVelocity = MathUtil.clamp(targetVelocity, -kNoLoadRpm, kNoLoadRpm);

        cmdLog.publish( "velocity", velocity );

        m_elevatorClosedLoopController.setReference(targetVelocity, ControlType.kVelocity);

    }

    public double getVelocity()
    {
        return m_elevatorEncoder.getVelocity();
    }

    public double getPosition()
    {
        return m_elevatorEncoder.getPosition();
    }

    public double getCurrent()
    {
        return m_masterMotor.getOutputCurrent();
    }

    public void logValues()
    {
        tlmLog.publish( "velocity", m_elevatorEncoder.getVelocity() );
        tlmLog.publish( "position", m_elevatorEncoder.getPosition() );
        tlmLog.publish( "current",  m_masterMotor.getOutputCurrent() );
    }

}
