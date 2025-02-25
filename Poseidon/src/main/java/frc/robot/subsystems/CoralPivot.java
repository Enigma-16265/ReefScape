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
public class CoralPivot extends SubsystemBase {
    public static final int kPivotMotorCanId     = 23;
    public static final double kPivotGearRatio   = 1.0 / 60.0;
    public static final double kNoLoadRpm        = 5500 * kPivotGearRatio;
    public static final double kMinRotPos        = 0.0;
    public static final double kMaxRotPos        = 20.0;
    public static final double kCurrentThreshold = 20.0;
    
    // PID tuning parameters for position control (to be tuned)
    private static final double kP_pos = 0.0;
    private static final double kI_pos = 0.0;
    private static final double kD_pos = 0.0;
   
    // Flags to enable/disable safety checks
    private boolean encoderCheckEnabled = true;
    private boolean currentCheckEnabled = true;
    
    private final SparkMax m_pivotSparkMax;
    public final SparkMaxConfig m_pivotSparkMaxConfig;
    
    private final RelativeEncoder m_pivotEncoder;
    private final SparkClosedLoopController m_pivotClosedLoopController;
    
    public CoralPivot() {
        m_pivotSparkMaxConfig = new SparkMaxConfig();
        // Configure conversion factors: position conversion factor accounts for gear reduction.
        m_pivotSparkMaxConfig.encoder.positionConversionFactor(kPivotGearRatio);
        // Velocity conversion: raw rotations per second * 60 = RPM, then account for gear reduction.
        m_pivotSparkMaxConfig.encoder.velocityConversionFactor(kPivotGearRatio * 60.0);
        
        // Configure PID parameters and output limits.
        m_pivotSparkMaxConfig.closedLoop.pid(kP_pos, kI_pos, kD_pos);
        m_pivotSparkMaxConfig.closedLoop.outputRange(-1.0, 1.0);
        
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
     * Sets the pivot position using the PID controller while enforcing safety limits.
     * The method applies safety checks (encoder and current draw) and then clamps the final
     * target between kMinPosition and kMaxPosition just before commanding the motor.
     *
     * @param position the desired pivot position (in rotations)
     */
    public void setPosition(double position) {
        double targetPosition = position;
        
        if (encoderCheckEnabled) {
            double currentPosition = m_pivotEncoder.getPosition();
            // Prevent driving further past the physical limits.
            if (targetPosition > currentPosition && currentPosition >= kMaxRotPos) {
                targetPosition = currentPosition;
            }
            if (targetPosition < currentPosition && currentPosition <= kMinRotPos) {
                targetPosition = currentPosition;
            }
        }
        
        if (currentCheckEnabled) {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                targetPosition = m_pivotEncoder.getPosition();
            }
        }
        
        // Clamp the final target position just before commanding the motor.
        targetPosition = MathUtil.clamp(targetPosition, kMinRotPos, kMaxRotPos);
        m_pivotClosedLoopController.setReference(targetPosition, ControlType.kPosition);
    }
    
    /**
     * Sets the pivot velocity using the PID controller while enforcing current draw safety.
     * If the current draw exceeds the threshold, the motor is commanded to 0 velocity.
     * The final velocity is clamped between -kNoLoadRpm and kNoLoadRpm (computed here) 
     * just before commanding the motor.
     *
     * @param velocity the desired pivot velocity (in RPM)
     */
    public void setVelocity(double velocity) {
        double targetVelocity = velocity;
        
        if (currentCheckEnabled) {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                targetVelocity = 0.0;
            }
        }
        
        // Compute kNoLoadRpm from the pivot's gear ratio for clamping.
        double kNoLoadRpm = 5500 * kPivotGearRatio;
        targetVelocity = MathUtil.clamp(targetVelocity, -kNoLoadRpm, kNoLoadRpm);
        m_pivotClosedLoopController.setReference(targetVelocity, ControlType.kVelocity);
    }
    
    /**
     * Directly sets the motor output while enforcing a current draw safety check.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (currentCheckEnabled) {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                m_pivotSparkMax.set(0.0);
                return;
            }
        }
        m_pivotSparkMax.set(speed);
    }
    
    public double getSpeedRPM() {
        return m_pivotEncoder.getVelocity();
    }
    
    public double getEncoderPosition() {
        return m_pivotEncoder.getPosition();
    }
    
    public double getCurrent() {
        return m_pivotSparkMax.getOutputCurrent();
    }
}
