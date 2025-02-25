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
public class AlgaeIntake extends SubsystemBase {
    public static final int    kIntakeMotorCanId = 20;
    public static final double kIntakeGearRatio  = 1.0 / 5.0;
    public static final double kNoLoadRpm        = 5500 * kIntakeGearRatio;
    public static final double kMinRotPos        = 0.0;
    public static final double kMaxRotPos        = 20.0;
    public static final double kCurrentThreshold = 20.0;

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Flags to enable/disable safety checks
    private boolean currentCheckEnabled = false;

    private final SparkMax       m_intakeSparkMax; 
    public final SparkMaxConfig m_intakeSparkMaxConfig;

    private final RelativeEncoder           m_intakeEncoder;
    private final SparkClosedLoopController m_intakePIDController;

    public AlgaeIntake() {
        m_intakeSparkMaxConfig = new SparkMaxConfig();
        // Configure the encoder for position conversion if needed.
        m_intakeSparkMaxConfig.encoder.positionConversionFactor(kIntakeGearRatio);
        // Configure the encoder to report velocity in RPM.
        // For instance, if the encoder reports rotations per second, multiplying by 60 converts to RPM.
        m_intakeSparkMaxConfig.encoder.velocityConversionFactor(kIntakeGearRatio * 60.0);

        // Set PID parameters and output limits for the closed-loop controller.
        m_intakeSparkMaxConfig.closedLoop.pid(kP, kI, kD);
        m_intakeSparkMaxConfig.closedLoop.outputRange(-1.0, 1.0);

        m_intakeSparkMax = new SparkMax(kIntakeMotorCanId, MotorType.kBrushless);
        m_intakeSparkMax.configure(m_intakeSparkMaxConfig, null, null);

        m_intakeEncoder = m_intakeSparkMax.getEncoder();
        m_intakeEncoder.setPosition(0.0);

        // Obtain the closed-loop controller from the SparkMax.
        m_intakePIDController = m_intakeSparkMax.getClosedLoopController();
    }

    /**
     * Directly sets the motor output without PID control.
     * If current checking is enabled and current draw exceeds the threshold, the motor is stopped.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                m_intakeSparkMax.set(0.0);
                return;
            }
        }
        m_intakeSparkMax.set(speed);
    }

    /**
     * Sets the intake velocity in RPM using the PID controller.
     * If current checking is enabled and the current draw is too high,
     * the target velocity is set to 0 RPM. The final value is then clamped
     * between -kNoLoadRpm and kNoLoadRpm.
     *
     * @param rpm the desired intake velocity in RPM
     */
    public void setVelocity(double rpm) {
        double targetRPM = rpm;
        
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                targetRPM = 0.0;
            }
        }
        
        targetRPM = MathUtil.clamp(targetRPM, -kNoLoadRpm, kNoLoadRpm);
        m_intakePIDController.setReference(targetRPM, ControlType.kVelocity);
    }

    /**
     * Sets the intake position in revolutions using the PID controller.
     * If current checking is enabled and the current draw is too high,
     * the target position is set to the current encoder position.
     * The final value is then clamped between kMinRotPos and kMaxRotPos.
     *
     * @param position the desired intake position (in revolutions)
     */
    public void setPosition(double position) {
        double targetPosition = position;
        
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                targetPosition = m_intakeEncoder.getPosition();
            }
        }
        
        targetPosition = MathUtil.clamp(targetPosition, kMinRotPos, kMaxRotPos);
        m_intakePIDController.setReference(targetPosition, ControlType.kPosition);
    }


    public double getSpeedRPM() {
        return m_intakeEncoder.getVelocity();
    }

    public double getEncoderPosition() {
        return m_intakeEncoder.getPosition();
    }

    public double getCurrent() {
        return m_intakeSparkMax.getOutputCurrent();
    }
    
}
