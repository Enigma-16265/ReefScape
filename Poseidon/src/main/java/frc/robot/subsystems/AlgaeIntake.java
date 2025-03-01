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
import frc.logging.DataNetworkTableLog;
//import edu.wpi.first.epilogue.Logged;

//@Logged
public class AlgaeIntake extends SubsystemBase
{
    private static final DataNetworkTableLog tlmLog =
        new DataNetworkTableLog( 
            "Subsystems.AlgaeIntake.tlm",
            Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "current", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final DataNetworkTableLog cmdLog =
    new DataNetworkTableLog( 
        "Subsystems.AlgaeIntake.cmd",
        Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    public static final int    kIntakeMotorCanId = 20;
    public static final double kIntakeGearRatio  = 1.0 / 5.0;
    public static final double kNoLoadRpm        = 5500 * kIntakeGearRatio;
    public static final double kCurrentThreshold = 20.0;

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Flag to enable/disable current draw safety check
    private boolean currentCheckEnabled = false;

    private final SparkMax       m_intakeSparkMax; 
    private final SparkMaxConfig m_intakeSparkMaxConfig;

    private final RelativeEncoder           m_intakeEncoder;
    private final SparkClosedLoopController m_intakePIDController;

    public AlgaeIntake() {
        m_intakeSparkMaxConfig = new SparkMaxConfig();
        // Configure the encoder for position conversion if needed.
        m_intakeSparkMaxConfig.encoder.positionConversionFactor(kIntakeGearRatio);
        // Configure the encoder to report velocity in RPM.
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
     * Enables or disables the current draw safety check.
     *
     * @param enabled true to enable, false to disable.
     */
    public void setCurrentCheckEnabled(boolean enabled) {
        currentCheckEnabled = enabled;
    }

    /**
     * Directly sets the motor output without PID control.
     * If current checking is enabled and current draw exceeds the threshold, the motor is stopped.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed( double speed )
    {
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                m_intakeSparkMax.set(0.0);
                return;
            }
        }

        cmdLog.publish( "speed", speed );

        m_intakeSparkMax.set( speed );
    }

    /**
     * Sets the intake position using the PID controller while enforcing a current draw safety check.
     * For a roller mechanism, no encoder position check is performed. The desired position is clamped 
     * between kMinPos and kMaxPos, and if the current draw is too high, the command is overridden 
     * to hold the current position.
     *
     * @param position the desired intake position (in rotations)
     */
    public void setPosition(double position) {
        double safePosition = position;
    
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                // If current is too high, hold the current position.
                safePosition = m_intakeEncoder.getPosition();
            }
        }

        cmdLog.publish( "position", position );
        
        m_intakePIDController.setReference(safePosition, ControlType.kPosition);
    }  

    /**
     * Sets the intake velocity in RPM using the PID controller.
     * If current checking is enabled and the current draw is too high,
     * the target velocity is set to 0 RPM. The final value is then clamped
     * between -kNoLoadRpm and kNoLoadRpm.
     *
     * @param velocity the desired intake velocity in RPM
     */
    public void setVelocity(double velocity) {
        double targetRPM = velocity;
        
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                targetRPM = 0.0;
            }
        }
        
        targetRPM = MathUtil.clamp(targetRPM, -kNoLoadRpm, kNoLoadRpm);

        cmdLog.publish( "velocity", velocity );

        m_intakePIDController.setReference(targetRPM, ControlType.kVelocity);
    }

    public double getVelocity() {
        return m_intakeEncoder.getVelocity();
    }

    public double getPosition() {
        return m_intakeEncoder.getPosition();
    }

    public double getCurrent() {
        return m_intakeSparkMax.getOutputCurrent();
    }

    public void logValues()
    {
        tlmLog.publish( "velocity", m_intakeEncoder.getVelocity() );
        tlmLog.publish( "position", m_intakeEncoder.getPosition() );
        tlmLog.publish( "current",  m_intakeSparkMax.getOutputCurrent() );
    }

}
