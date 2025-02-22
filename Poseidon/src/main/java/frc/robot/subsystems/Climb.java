package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    // Adjust this CAN ID and gear ratio for your climb mechanism.
    public static final int    kClimbMotorCanId = 7;
    public static final double kClimbGearRatio  = 1.0 / 100.0; // Example: 1:1 gear ratio
    public static final double kMaxClimbRPM     = 5000; // Example maximum RPM under no-load

    // PID tuning parameters (to be tuned later)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private final SparkMax       m_climbSparkMax;
    private final SparkMaxConfig m_climbSparkMaxConfig;

    private final RelativeEncoder           m_climbEncoder;
    private final SparkClosedLoopController m_climbPIDController;

    public Climb() {
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
     * Sets the climb speed in RPM using the PID controller.
     * The target RPM is clamped to be within ±kMaxClimbRPM.
     *
     * @param rpm the desired climb speed in RPM
     */
    public void setClimbSpeed(double rpm) {
        double cappedRPM = MathUtil.clamp(rpm, -kMaxClimbRPM, kMaxClimbRPM);
        m_climbPIDController.setReference(cappedRPM, ControlType.kVelocity);
    }

    /**
     * Directly sets the motor output without PID control.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        m_climbSparkMax.set(speed);
    }
}
