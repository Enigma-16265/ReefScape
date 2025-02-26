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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
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

  // ------------------
  // Command Members (Controller 2)
  // ------------------
  Command algaeIntakeCommand;
  Command algaePivotLowPositionCommand;
  Command algaePivotHighPositionCommand;
  Command elevatorCommand;
  Command coralPivotCommand;
  Command coralIntakeIntakeCommand;
  Command coralIntakeOuttakeCommand;
  Command climbUpCommand;
  Command climbDownCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {

    // ------------------
    // Initialize Commands for Controller 2
    // ------------------
    // AlgaePivot: Left bumper sets position to 25 revs; Right bumper sets position to 0 revs.
    algaePivotLowPositionCommand = new InstantCommand(
        () -> algaePivot.setPosition(25.0), algaePivot);
    algaePivotHighPositionCommand = new InstantCommand(
        () -> algaePivot.setPosition(0.0), algaePivot);

    // AlgaeIntake: Use left/right trigger difference as speed input.
    algaeIntakeCommand = new frc.robot.commands.algae_intake.AlgaeIntakeCommand(
        algaeIntake,
        () -> mechanicXbox.getLeftTriggerAxis() - mechanicXbox.getRightTriggerAxis());

    // Elevator: Use left thumbstick Y axis for elevator speed.
    elevatorCommand = new frc.robot.commands.elevator.ElevatorCommand(
        elevator,
        () -> mechanicXbox.getLeftY());

    // CoralPivot: Use right thumbstick Y axis for pivot speed.
    coralPivotCommand = new frc.robot.commands.coral_pivot.CoralPivotCommand(
        coralPivot,
        () -> mechanicXbox.getRightY());

    // CoralIntake: X button provides constant intake speed (e.g. +0.5), A button constant outtake speed (-0.5).
    coralIntakeIntakeCommand = new RunCommand(
        () -> coralIntake.setSpeed(0.5), coralIntake);
    coralIntakeOuttakeCommand = new RunCommand(
        () -> coralIntake.setSpeed(-0.5), coralIntake);

    // Climb: Y button provides constant climbing speed (+0.5), B button provides constant lowering speed (-0.5).
    climbUpCommand = new RunCommand(
        () -> climb.setSpeed(0.5), climb);
    climbDownCommand = new RunCommand(
        () -> climb.setSpeed(-0.5), climb);

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
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }

    // Controller 2 mapping
    // AlgaePivot  - L/R bumper  : R set's position in Revs to 0, L set's position in Revs to 25
    // AlgaeIntake - L/R trigger : L provides speed to intake, R provides speed to outtake
    // Elevator    - Left Thumb  : Up provides speed to lift, Down provides speed to drop
    // CoralPivot  - Right Thumb : Up provides speed to rotate forward, Down provides speed to rotate backwards
    // CoralIntake - X/A buttons : X provides constant speed to intake, A provides constant speed to outtake
    // Climb       - Y/B buttons : Y provides constant speed to climb, B provides constant speed to let down

    // ------------------
    // Bind Controller 2 Buttons to Commands
    // ------------------

    // AlgaePivot preset positions: 
    // Left bumper sets pivot to 25 revolutions, Right bumper sets pivot to 0.
    mechanicXbox.leftBumper().onTrue(algaePivotLowPositionCommand);
    mechanicXbox.rightBumper().onTrue(algaePivotHighPositionCommand);

    // AlgaeIntake: Left and right triggers control intake/outtake speed.
    mechanicXbox.leftTrigger().whileTrue(algaeIntakeCommand);
    mechanicXbox.rightTrigger().whileTrue(algaeIntakeCommand);

    // Elevator: Set default command to continuously control elevator speed.
    elevator.setDefaultCommand(elevatorCommand);

    // CoralPivot: Set default command to continuously control pivot speed.
    coralPivot.setDefaultCommand(coralPivotCommand);

    // CoralIntake: 
    // X button for intake at constant speed, A button for outtake at constant speed.
    mechanicXbox.x().whileTrue(coralIntakeIntakeCommand);
    mechanicXbox.a().whileTrue(coralIntakeOuttakeCommand);

    // Climb: 
    // Y button for climbing up, B button for lowering.
    mechanicXbox.y().whileTrue(climbUpCommand);
    mechanicXbox.b().whileTrue(climbDownCommand);

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
}
