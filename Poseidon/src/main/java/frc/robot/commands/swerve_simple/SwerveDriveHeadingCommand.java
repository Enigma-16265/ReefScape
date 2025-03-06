package frc.robot.commands.swerve_simple;

import frc.robot.subsystems.SwerveSimple;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import swervelib.math.SwerveMath; // Assumes SwerveMath is available for scaling

public class SwerveDriveHeadingCommand extends Command {
  private final SwerveSimple swerve;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier headingX;
  private final DoubleSupplier headingY;

  public SwerveDriveHeadingCommand(SwerveSimple swerve,
                                   DoubleSupplier translationX,
                                   DoubleSupplier translationY,
                                   DoubleSupplier headingX,
                                   DoubleSupplier headingY) {
    this.swerve = swerve;
    this.translationX = translationX;
    this.translationY = translationY;
    this.headingX = headingX;
    this.headingY = headingY;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // Create a Translation2d from the raw translation inputs.
    Translation2d inputTranslation = new Translation2d(
        translationX.getAsDouble(), 
        translationY.getAsDouble());
    
    // Scale the translation inputs (for example, using a factor of 0.8).
    Translation2d scaledInputs = SwerveMath.scaleTranslation(inputTranslation, 0.8);

    // Retrieve current heading (in radians) and maximum velocity from the underlying swerve drive.
    double currentHeadingRadians = swerve.swerveDrive.getOdometryHeading().getRadians();
    double maxVelocity = swerve.swerveDrive.getMaximumChassisVelocity();
    
    // Use the swerve controller to compute target speeds.
    // The controller uses the scaled translational inputs and the heading setpoint (derived from headingX and headingY)
    var targetSpeeds = swerve.swerveDrive.swerveController.getTargetSpeeds(
          scaledInputs.getX(),
          scaledInputs.getY(),
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          currentHeadingRadians,
          maxVelocity);

    // Command the drive using field-relative control with the computed target speeds.
    swerve.swerveDrive.driveFieldOriented(targetSpeeds);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
