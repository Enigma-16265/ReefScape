package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.epilogue.Logged;

@Logged
public class Climb extends SubsystemBase
{
    // Adjust this CAN ID and gear ratio for your climb mechanism.
    public static final int    kClimbMotorCanId = 7;
    public static final double kClimbGearRatio  = 1.0 / 100.0; // Example: 1:1 gear ratio
    public static final double kNoLoadRpm        = 5500 * kClimbGearRatio;
    public static final double kMinRotPos        = 0.0;
    public static final double kMaxRotPos        = 20.0;
    public static final double kCurrentThreshold = 20.0;

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Flags to enable/disable safety checks
    private boolean encoderCheckEnabled = false;
    private boolean currentCheckEnabled = false;

    private final SparkMax       m_climbSparkMax;
    private final SparkMaxConfig m_climbSparkMaxConfig;

    private final RelativeEncoder           m_climbEncoder;
    private final SparkClosedLoopController m_climbPIDController;

    public Climb()
    {
        m_climbSparkMaxConfig = new SparkMaxConfig();
        // Configure the encoder conversion factors.
        m_climbSparkMaxConfig.encoder.positionConversionFactor(kClimbGearRatio);
        m_climbSparkMaxConfig.encoder.velocityConversionFactor(kClimbGearRatio * 60.0);

        // Set PID parameters and output limits.
        m_climbSparkMaxConfig.closedLoop.pid(kP, kI, kD);
        m_climbSparkMaxConfig.closedLoop.outputRange(-1.0, 1.0);

        m_climbSparkMax = new SparkMax(kClimbMotorCanId, MotorType.kBrushless);
        m_climbSparkMax.configure(m_climbSparkMaxConfig, null, null);

        m_climbEncoder = m_climbSparkMax.getEncoder();
        m_climbEncoder.setPosition(0.0);

        // Obtain the closed-loop controller for velocity control.
        m_climbPIDController = m_climbSparkMax.getClosedLoopController();
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
            double currentDraw = m_climbSparkMax.getOutputCurrent();
            if ( currentDraw > kCurrentThreshold )
            {
                m_climbSparkMax.set( 0.0 );
                return;
            }
        }
        m_climbSparkMax.set( speed );
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
            double currentPosition = m_climbEncoder.getPosition();
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
            double currentDraw = m_climbSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                // Hold the current position when current is too high.
                targetPosition = m_climbEncoder.getPosition();
            }
        }
    
        // Clamp the target position just before commanding the motor.
        targetPosition = MathUtil.clamp(targetPosition, kMinRotPos, kMaxRotPos);
        m_climbPIDController.setReference(targetPosition, ControlType.kPosition);
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
            double currentDraw = m_climbSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold)
            {
                targetVelocity = 0.0;
            }
        }
        
        // Clamp the final velocity just before commanding the motor.
        targetVelocity = MathUtil.clamp(targetVelocity, -kNoLoadRpm, kNoLoadRpm);
        m_climbPIDController.setReference(targetVelocity, ControlType.kVelocity);
    }

    public double getSpeedRPM()
    {
        return m_climbEncoder.getVelocity();
    }

    public double getEncoderPosition()
    {
        return m_climbEncoder.getPosition();
    }

    public double getCurrent()
    {
        return m_climbSparkMax.getOutputCurrent();
    }

}
