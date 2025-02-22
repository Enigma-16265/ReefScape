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
public class AlgaeIntake extends SubsystemBase
{
    public static final int    kIntakeMotorCanId = 20;
    public static final double kIntakeGearRatio  = 1.0 / 5.0;
    public static final double kNoLoadRpm        = 5500 * kIntakeGearRatio;

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final SparkMax       m_intakeSparkMax; 
    private final SparkMaxConfig m_intakeSparkMaxConfig;

    private final RelativeEncoder           m_intakeEncoder;
    private final SparkClosedLoopController m_intakePIDController;

    public AlgaeIntake()
    {
        m_intakeSparkMaxConfig = new SparkMaxConfig();
        // Configure the encoder for position conversion if needed
        m_intakeSparkMaxConfig.encoder.positionConversionFactor( kIntakeGearRatio );
        // Configure the encoder to report velocity in RPM.
        // For instance, if the encoder reports rotations per second, multiplying by 60 converts to RPM.
        m_intakeSparkMaxConfig.encoder.velocityConversionFactor( kIntakeGearRatio * 60.0 );

        // Set PID parameters and output limits for the closed-loop controller.
        m_intakeSparkMaxConfig.closedLoop.pid( kP, kI, kD );
        m_intakeSparkMaxConfig.closedLoop.outputRange( -1.0, 1.0 );

        m_intakeSparkMax = new SparkMax( kIntakeMotorCanId, MotorType.kBrushless );
        m_intakeSparkMax.configure( m_intakeSparkMaxConfig, null, null );

        m_intakeEncoder = m_intakeSparkMax.getEncoder();
        m_intakeEncoder.setPosition( 0.0 );

        // Obtain the closed-loop controller from the SparkMax
        m_intakePIDController = m_intakeSparkMax.getClosedLoopController();
    }

    /**
     * Sets the intake speed in RPM using the PID controller on the SparkMax.
     *
     * @param rpm the desired intake speed in RPM
     */
    public void setIntakeSpeed( double rpm )
    {
        double cappedRPM = MathUtil.clamp( rpm, -kNoLoadRpm, kNoLoadRpm );
        m_intakePIDController.setReference( cappedRPM, ControlType.kVelocity );
    }

    /**
     * Directly sets the motor output without PID control.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed( double speed )
    {
        m_intakeSparkMax.set( speed );
    }

    public double getSpeedRPM()
    {
        return m_intakeEncoder.getVelocity();
    }

    public double getEncoderPosition()
    {
        return m_intakeEncoder.getPosition();
    }
}
