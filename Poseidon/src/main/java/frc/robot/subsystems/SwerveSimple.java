package frc.robot.subsystems;

import java.io.File;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSimple extends SubsystemBase
{
    double maximumSpeed = Units.feetToMeters(4.5);
    File swerveJsonDirectory = new File( Filesystem.getDeployDirectory(), "swerve/neo");
    public SwerveDrive  swerveDrive;

    public SwerveSimple()
    {
        try
        {
            swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeed);
        }
        catch( Exception e )
        {
            // do nothing
        }
    }

    public void drive( Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop )
    {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public void driveFieldOriented(ChassisSpeeds fieldRelativeSpeeds )
    {
        swerveDrive.driveFieldOriented( fieldRelativeSpeeds );
    }

    public double getMaximumChassisVelocity()
    {
        return swerveDrive.getMaximumChassisVelocity();
    }

    public double getMaximumChassisAngularVelocity()
    {
        return swerveDrive.getMaximumChassisAngularVelocity();
    }

}
