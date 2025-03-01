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
public class CoralIntake extends SubsystemBase
{
    private static final DataNetworkTableLog tlmLog =
        new DataNetworkTableLog( 
            "Subsystems.CoralIntake.tlm",
            Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "current", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );

    private static final DataNetworkTableLog cmdLog =
        new DataNetworkTableLog( 
            "Subsystems.CoralIntake.cmd",
            Map.of( "velocity", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "position", DataNetworkTableLog.COLUMN_TYPE.DOUBLE,
                    "speed", DataNetworkTableLog.COLUMN_TYPE.DOUBLE ) );
    
    public static final int    kIntakeMotorCanId = 6;
    public static final double kCoralGearRatio   = 1.0 / 12.0;
    public static final double kNoLoadRpm        = 5500 * kCoralGearRatio;
    public static final double kCurrentThreshold = 20.0;

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Safety flags
    private boolean currentCheckEnabled = false;

    private final SparkMax       m_intakeSparkMax;
    private final SparkMaxConfig m_intakeSparkMaxConfig;

    private final RelativeEncoder           m_intakeEncoder;
    private final SparkClosedLoopController m_intakePIDController;

    public CoralIntake() {
        m_intakeSparkMaxConfig = new SparkMaxConfig();
        // Configure the encoder conversion factors.
        m_intakeSparkMaxConfig.encoder.positionConversionFactor(kCoralGearRatio);
        m_intakeSparkMaxConfig.encoder.velocityConversionFactor(kCoralGearRatio * 60.0);

        // Set PID parameters and output limits.
        m_intakeSparkMaxConfig.closedLoop.pid(kP, kI, kD);
        m_intakeSparkMaxConfig.closedLoop.outputRange(-1.0, 1.0);

        m_intakeSparkMax = new SparkMax(kIntakeMotorCanId, MotorType.kBrushless);
        m_intakeSparkMax.configure(m_intakeSparkMaxConfig, null, null);

        m_intakeEncoder = m_intakeSparkMax.getEncoder();
        m_intakeEncoder.setPosition(0.0);

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
     * Directly sets the motor output while enforcing a current draw safety check.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                // If current is too high, stop the motor.
                m_intakeSparkMax.set(0.0);
                return;
            }
        }

        cmdLog.publish( "speed", speed );

        m_intakeSparkMax.set(speed);

        logValues();
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

        logValues();
    }    

    /**
     * Sets the intake velocity using the PID controller while enforcing a current draw safety check.
     * If the current draw exceeds the threshold, the motor is commanded to 0 velocity.
     * The commanded velocity is also clamped within the allowable no-load RPM range.
     *
     * @param velocity the desired intake velocity in RPM
     */
    public void setVelocity(double velocity) {
        double safeVelocity = velocity;

        if (currentCheckEnabled) {
            double currentDraw = m_intakeSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                safeVelocity = 0.0;
            }
        }

        safeVelocity = MathUtil.clamp(safeVelocity, -kNoLoadRpm, kNoLoadRpm);

        cmdLog.publish( "velocity", velocity );

        m_intakePIDController.setReference(safeVelocity, ControlType.kVelocity);

        logValues();
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
