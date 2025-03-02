// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.algae_intake.AlgaeIntakeDutyCommand;
import frc.robot.commands.algae_pivot.AlgaePivotPositionCommand;
import frc.robot.commands.climb.ClimbHoldCommand;
import frc.robot.commands.coral_intake.CoralIntakeHoldCommand;
import frc.robot.commands.coral_pivot.CoralPivotHoldCommand;
import frc.robot.commands.elevator.ElevatorHoldCommand;
import frc.robot.subsystems.AlgaeIntake;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CoralIntake;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  final CommandXboxController mechanicXbox = new CommandXboxController(1);

  // Subsystems
  AlgaeIntake algaeIntake = new AlgaeIntake();
  AlgaePivot  algaePivot  = new AlgaePivot();
  Climb       climb       = new Climb();
  CoralIntake coralIntake = new CoralIntake();
  CoralPivot  coralPivot  = new CoralPivot();
  Elevator    elevator    = new Elevator();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {

    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      // driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

    configureMechanicsBindings();

  }

  // Controller 2 mapping
  // AlgaePivot  - L/R bumper  : R set's position in Revs to 0, L set's position in Revs to 25
  // AlgaeIntake - L/R trigger : L provides speed to intake, R provides speed to outtake
  // Elevator    - Left Thumb  : Up provides speed to lift, Down provides speed to drop
  // CoralPivot  - Right Thumb : Up provides speed to rotate forward, Down provides speed to rotate backwards
  // CoralIntake - X/A buttons : X provides constant speed to intake, A provides constant speed to outtake
  // Climb       - Y/B buttons : Y provides constant speed to climb, B provides constant speed to let down
  void configureMechanicsBindings() {
      // AlgaePivot preset positions: 
      // Left bumper sets pivot to 25 revolutions, Right bumper sets pivot to 0.
      mechanicXbox.leftBumper().onTrue(new AlgaePivotPositionCommand(algaePivot, 25.0));
      mechanicXbox.rightBumper().onTrue(new AlgaePivotPositionCommand(algaePivot, 0.0));

      // AlgaeIntake: Left and right triggers control intake/outtake speed.
      mechanicXbox.leftTrigger().whileTrue(new AlgaeIntakeDutyCommand(
          algaeIntake, () -> mechanicXbox.getLeftTriggerAxis() - mechanicXbox.getRightTriggerAxis()));
      mechanicXbox.rightTrigger().whileTrue(new AlgaeIntakeDutyCommand(
          algaeIntake, () -> mechanicXbox.getLeftTriggerAxis() - mechanicXbox.getRightTriggerAxis()));

      // Elevator: Set default command to continuously control elevator speed.
      elevator.setDefaultCommand(new ElevatorHoldCommand(elevator, () -> mechanicXbox.getLeftY()));

      // CoralPivot: Set default command to continuously control pivot speed.
      coralPivot.setDefaultCommand(new CoralPivotHoldCommand(coralPivot, () -> mechanicXbox.getRightY()));

      // CoralIntake: X button for intake at constant speed, A button for outtake at constant speed.
      coralIntake.setDefaultCommand(new CoralIntakeHoldCommand(
          coralIntake,
          mechanicXbox.x()::getAsBoolean,
          mechanicXbox.a()::getAsBoolean,
          0.5,
          -0.5));

      // Climb: Y button for climbing up, B button for lowering.
      climb.setDefaultCommand(new ClimbHoldCommand(
          climb,
          mechanicXbox.y()::getAsBoolean,
          mechanicXbox.b()::getAsBoolean,
          0.5,
          -0.5));
  }

  public void configureMechanicsTestBindings()
  {

    // Bind the duty cycle command for each subsystem to the Xbox Controller 2 thumbstick Y-axis.
    // Uncomment ONE of the following lines when you want to test that specific subsystem.

    algaeIntake.setDefaultCommand(
        new frc.robot.commands.algae_intake.AlgaeIntakeDutyCommand(
            algaeIntake, 
            () -> mechanicXbox.getRightY()
        )
    );

    algaePivot.setDefaultCommand(
        new frc.robot.commands.algae_pivot.AlgaePivotDutyCommand(
            algaePivot, 
            () -> mechanicXbox.getRightY()
        )
    );

    climb.setDefaultCommand(
        new frc.robot.commands.climb.ClimbDutyCommand(
            climb, 
            () -> mechanicXbox.getRightY()
        )
    );

    coralIntake.setDefaultCommand(
        new frc.robot.commands.coral_intake.CoralIntakeDutyCommand(
            coralIntake, 
            () -> mechanicXbox.getRightY()
        )
    );

    coralPivot.setDefaultCommand(
        new frc.robot.commands.coral_pivot.CoralPivotDutyCommand(
            coralPivot, 
            () -> mechanicXbox.getRightY()
        )
    );

    elevator.setDefaultCommand(
        new frc.robot.commands.elevator.ElevatorDutyCommand(
            elevator, 
            () -> mechanicXbox.getRightY()
        )
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public void logValues()
  {
    algaeIntake.logValues();
    algaePivot.logValues();
    climb.logValues();
    coralIntake.logValues();
    coralPivot.logValues();
    elevator.logValues();
  }

}
