package frc.robot.commands.swerve_simple;

import frc.robot.subsystems.SwerveSimple;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends Command {
  private final SwerveSimple swerve;
  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier angularRotation;
  
  // You can adjust this constant as needed for your robot
  private final double maxAngularVelocity = Math.PI; 

  public SwerveDriveCommand(SwerveSimple swerve, 
                            DoubleSupplier translationX, 
                            DoubleSupplier translationY, 
                            DoubleSupplier angularRotation) {
    this.swerve = swerve;
    this.translationX = translationX;
    this.translationY = translationY;
    this.angularRotation = angularRotation;
    addRequirements(swerve);
  }

  @Override
  public void execute() {
    // Retrieve the maximum chassis velocities from the underlying swerve drive.
    double maxChassisVelocity = swerve.getMaximumChassisVelocity();
    double maxChassisAngularVelocity = swerve.getMaximumChassisAngularVelocity();

    // Scale the normalized inputs by the maximum velocities.
    Translation2d translation = new Translation2d(
        translationX.getAsDouble() * maxChassisVelocity,
        translationY.getAsDouble() * maxChassisVelocity);
    
    double rotation = angularRotation.getAsDouble() * maxChassisAngularVelocity;
    
    // Command the drive subsystem using field-relative control and closed-loop velocity.
    swerve.drive(translation, rotation, true, false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
