package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralPivot extends SubsystemBase {
    public static final int kPivotMotorCanId = 23;
    // With a 66:1 gear reduction, the output is 1/66th of the motor's rotations.
    public static final double kPivotGearRatio = 1.0 / 66.0;

    // PID tuning parameters for position control (to be tuned)
    private static final double kP_pos = 0.0;
    private static final double kI_pos = 0.0;
    private static final double kD_pos = 0.0;

    // Safety limits for pivot position (in output rotations after gearing)
    private static final double kMinPosition = -1.0; // adjust as needed
    private static final double kMaxPosition = 1.0;  // adjust as needed

    // Current threshold in amps to protect the gears
    private static final double kCurrentThreshold = 20.0;

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
     * This method clamps the desired position between kMinPosition and kMaxPosition.
     * When encoder checking is enabled, it verifies that the target position will not drive
     * the mechanism further past its physical limits. Similarly, if the current draw is too high,
     * it holds the current position.
     *
     * @param targetPosition the desired pivot position (in rotations)
     */
    public void setPivotPosition(double targetPosition) {
        // Clamp the target position to the overall safe range.
        double safeTarget = MathUtil.clamp(targetPosition, kMinPosition, kMaxPosition);

        if (encoderCheckEnabled) {
            double currentPosition = m_pivotEncoder.getPosition();
            // Prevent driving further past the physical limits.
            if (safeTarget > currentPosition && currentPosition >= kMaxPosition) {
                // Instead of commanding a 0 output, hold the current position.
                safeTarget = currentPosition;
            }
            if (safeTarget < currentPosition && currentPosition <= kMinPosition) {
                safeTarget = currentPosition;
            }
        }

        if (currentCheckEnabled) {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                // Hold the current position when current is too high.
                safeTarget = m_pivotEncoder.getPosition();
            }
        }

        // Command the motor to move to the (potentially adjusted) safe target using PID control.
        m_pivotClosedLoopController.setReference(safeTarget, ControlType.kPosition);
    }

    /**
     * Directly sets the motor output while enforcing a current draw safety check.
     *
     * @param speed the motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        // Check the current draw if safety is enabled.
        if (currentCheckEnabled) {
            double currentDraw = m_pivotSparkMax.getOutputCurrent();
            if (currentDraw > kCurrentThreshold) {
                // If the current is too high, stop the motor.
                m_pivotSparkMax.set(0.0);
                return;
            }
        }
        m_pivotSparkMax.set(speed);
    }
}
