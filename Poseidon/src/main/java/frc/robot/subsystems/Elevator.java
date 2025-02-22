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
public class Elevator extends SubsystemBase {
    public static final int kElevatorMasterCanId = 21;
    public static final int kElevatorFollowerCanId = 22;
    // With a 5:1 gear reduction, the output rotates at 1/5th of the motor's rotations.
    public static final double kElevatorGearRatio = 1.0 / 5.0;

    // PID tuning parameters for position control (to be tuned)
    private static final double kP = 0.0;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    // Safety limits for elevator position (in output rotations after gearing)
    private static final double kMinPosition = 0.0;  // adjust as needed
    private static final double kMaxPosition = 10.0; // adjust as needed

    // Current threshold in amps to protect the mechanism
    private static final double kCurrentThreshold = 20.0;

    // Flags to enable/disable safety checks
    private boolean encoderCheckEnabled = true;
    private boolean currentCheckEnabled = true;

    private final SparkMax m_masterMotor;
    private final SparkMax m_followerMotor;
    
    // Separate config objects for master and follower.
    public final SparkMaxConfig m_elevatorMasterConfig;
    public final SparkMaxConfig m_elevatorFollowerConfig;

    private final RelativeEncoder m_elevatorEncoder;
    private final SparkClosedLoopController m_elevatorClosedLoopController;

    public Elevator() {
        // Create and configure the master config.
        m_elevatorMasterConfig = new SparkMaxConfig();
        m_elevatorMasterConfig.encoder.positionConversionFactor(kElevatorGearRatio);
        m_elevatorMasterConfig.encoder.velocityConversionFactor(kElevatorGearRatio * 60.0);
        m_elevatorMasterConfig.closedLoop.pid(kP, kI, kD);
        m_elevatorMasterConfig.closedLoop.outputRange(-1.0, 1.0);

        // Create and configure the follower config.
        m_elevatorFollowerConfig = new SparkMaxConfig();
        // If you need the follower inverted relative to the master, set it here.
        m_elevatorFollowerConfig.inverted(true);
        m_elevatorFollowerConfig.follow( kElevatorFollowerCanId );

        m_masterMotor = new SparkMax(kElevatorMasterCanId, MotorType.kBrushless);
        m_masterMotor.configure(m_elevatorMasterConfig, null, null);

        m_followerMotor = new SparkMax(kElevatorFollowerCanId, MotorType.kBrushless);
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
     * Sets the elevator position using the PID controller while enforcing safety limits.
     * The desired position is clamped between kMinPosition and kMaxPosition.
     * When encoder checking is enabled, the target is compared to the current position,
     * and if the mechanism is already at a limit, the command is modified to hold position.
     *
     * @param targetPosition the desired elevator position (in rotations)
     */
    public void setElevatorPosition(double targetPosition) {
        // Clamp the target position to the overall safe range.
        double safeTarget = MathUtil.clamp(targetPosition, kMinPosition, kMaxPosition);

        if (encoderCheckEnabled) {
            double currentPosition = m_elevatorEncoder.getPosition();
            // Prevent commanding movement further past the safe limits.
            if (safeTarget > currentPosition && currentPosition >= kMaxPosition) {
                safeTarget = currentPosition;
            }
            if (safeTarget < currentPosition && currentPosition <= kMinPosition) {
                safeTarget = currentPosition;
            }
        }

        if (currentCheckEnabled) {
            double currentDraw = m_masterMotor.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                // If current is too high, hold the current position.
                safeTarget = m_elevatorEncoder.getPosition();
            }
        }

        // Command the master motor to move to the (potentially adjusted) safe target using PID control.
        m_elevatorClosedLoopController.setReference(safeTarget, ControlType.kPosition);
    }

    /**
     * Directly sets the master motor output while enforcing a current draw safety check.
     * The follower will mirror the master.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        if (currentCheckEnabled) {
            double currentDraw = m_masterMotor.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                m_masterMotor.set(0.0);
                return;
            }
        }
        m_masterMotor.set(speed);
    }

    public double getSpeedRPM()
    {
        return m_elevatorEncoder.getVelocity();
    }

    public double getEncoderPosition()
    {
        return m_elevatorEncoder.getPosition();
    }

}
