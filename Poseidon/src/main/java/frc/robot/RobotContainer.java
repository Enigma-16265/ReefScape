// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.algae_intake.AlgaeIntakeDutyCommand;
import frc.robot.commands.algae_pivot.AlgaePivotPositionCommand;
import frc.robot.commands.climb.ClimbPositionCommand;
import frc.robot.commands.coral_intake.CoralIntakeDutyCommand;
import frc.robot.commands.coral_pivot.CoralPivotPositionCommand;
import frc.robot.commands.elevator.ElevatorPositionCommand;
import frc.robot.commands.elevator.ElevatorPositionStopCommand;
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
  static final double DriveDefaultScale = 0.8;
  static final double DriveDefaultSlow  = 0.5;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final         CommandXboxController driverXbox = new CommandXboxController(0);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/neo"));

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of( drivebase.getSwerveDrive(),
                                                                 () -> driverXbox.getLeftY() * -1,
                                                                 () -> driverXbox.getLeftX() * -1 )
                                                            .withControllerRotationAxis( driverXbox::getRightX )
                                                            .deadband( OperatorConstants.DEADBAND )
                                                            .scaleTranslation( DriveDefaultScale )
                                                            .allianceRelativeControl( true );

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

  private void configureBindings()
  {

    configureDriverBindings();
    configureMechanicsBindings();
    //configureMechanicsTestBindings();

  }

  /*
    Driver:
    1. LT/RT - Algae Intake
    2. LB/RB - Algae Pivot
    3. X - Free
    4. A - Zero drive train Gyro
    5. B - Free
    6. Y - Free
    7. Start - Free
    8. Back - Free
    9. POV Up (DPad) - Hold Slow Mode
  */
  private void configureDriverBindings()
  {

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    drivebase.setDefaultCommand( driveFieldOrientedAnglularVelocity );

    driverXbox.a().onTrue( ( Commands.runOnce( drivebase::zeroGyro ) ) );

    // AlgaePivot preset positions: 
    // Left bumper sets pivot to 25 revolutions, Right bumper sets pivot to 0.
    driverXbox.leftBumper().onTrue(new AlgaePivotPositionCommand( algaePivot, 0.0 ) );
    driverXbox.rightBumper().onTrue(new AlgaePivotPositionCommand( algaePivot, 47.0 ) );

    // AlgaeIntake: Left and right triggers control intake/outtake speed.
    driverXbox.leftTrigger().whileTrue(new AlgaeIntakeDutyCommand(
        algaeIntake, () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis()));
    driverXbox.rightTrigger().whileTrue(new AlgaeIntakeDutyCommand(
        algaeIntake, () -> driverXbox.getLeftTriggerAxis() - driverXbox.getRightTriggerAxis()));    

    driverXbox.povUp().onTrue( new InstantCommand( () -> {
        driveAngularVelocity.scaleTranslation( DriveDefaultSlow );
      } )
    );

    driverXbox.povUp().onFalse( new InstantCommand( () -> {
        driveAngularVelocity.scaleTranslation( DriveDefaultScale );
      } )
    );

  }

  /*
    Mechanics:
    1. POV Right (DPad) - Elevator mid level
    2. POV Up (DPad) - Elevator high level
    3. POV Down (DPad) - Elevator low level
    4. X - Coral intake position
    5. B - Coral low position
    6. LT/RT - Coral Intake/Outake
    7. LB/RB - Pull Up/Pull Down the Climb
    8. Right Stick - Lift/Climb Adjustment
 */
  void configureMechanicsBindings()
  {

    // Elevator: Set default command to continuously control elevator speed.
    mechanicXbox.povRight().onTrue(
        new ParallelCommandGroup(
            new ElevatorPositionCommand( elevator, 81.3 ),
            new ConditionalCommand(
                new CoralPivotPositionCommand( coralPivot, 240.0 ), // runs if condition true
                new InstantCommand(() -> {},  coralPivot ),                        // does nothing if condition false
                () -> coralPivot.getPosition() > 215.0                             // lambda condition check
            )
        )
    );
      
    //mechanicXbox.povUp().onTrue( new ElevatorPositionCommand( elevator, 147.32 ) );
    mechanicXbox.povUp().onTrue(
        new ParallelCommandGroup(
            new ElevatorPositionCommand( elevator, 169.0 ),
            new ConditionalCommand(
                new CoralPivotPositionCommand( coralPivot, 240.0 ), // runs if condition true
                new InstantCommand(() -> {},  coralPivot ),                        // does nothing if condition false
                () -> coralPivot.getPosition() > 215.0                             // lambda condition check
            )
        )
    );

    mechanicXbox.povDown().onTrue(
        new ParallelCommandGroup(
            new ElevatorPositionStopCommand( elevator, 0.0, 5.0 ),
            new ConditionalCommand(
                new CoralPivotPositionCommand( coralPivot, 224.0 ), // runs if condition true (60)
                new InstantCommand(() -> {},  coralPivot ),                        // does nothing if condition false
                () -> coralPivot.getPosition() > 215.0                             // lambda condition check
            )
        )
    );

    // CoralPivot: Complete instantaneous commands to control pivot position in Degrees.
    mechanicXbox.x().onTrue( new CoralPivotPositionCommand( coralPivot, 60.0 ) );
    mechanicXbox.b().onTrue( new CoralPivotPositionCommand( coralPivot, 245.0 ) );

    // CoralIntake: X button for intake at constant speed, A button for outtake at constant speed.
    mechanicXbox.leftTrigger().whileTrue( new CoralIntakeDutyCommand(
      coralIntake, () -> mechanicXbox.getLeftTriggerAxis() * 0.5));
    
    mechanicXbox.rightTrigger().whileTrue(new CoralIntakeDutyCommand(
        coralIntake, () -> -mechanicXbox.getRightTriggerAxis() * 1.0));

    // Climb: Y button for climbing up, B button for lowering.
    climb.setDefaultCommand(
      new frc.robot.commands.climb.ClimbDutyCommand(
          climb, 
          () -> mechanicXbox.getRightY(),
          0.5
      )
    );

    mechanicXbox.leftBumper().onTrue( new ClimbPositionCommand( climb, 0 ) );
    mechanicXbox.rightBumper().onTrue( new ClimbPositionCommand( climb, 20 ) );

  }

  public void configureMechanicsTestBindings()
  {

    // Bind the duty cycle command for each subsystem to the Xbox Controller 2 thumbstick Y-axis.
    // Uncomment ONE of the following lines when you want to test that specific subsystem.

    // algaeIntake.setDefaultCommand(
    //     new frc.robot.commands.algae_intake.AlgaeIntakeDutyCommand(
    //         algaeIntake, 
    //         () -> mechanicXbox.getRightY()
    //     )
    // );

    // algaePivot.setDefaultCommand(
    //     new frc.robot.commands.algae_pivot.AlgaePivotDutyCommand(
    //         algaePivot, 
    //         () -> -mechanicXbox.getRightY(),
    //         0.1
    //     )
    // );

    // mechanicXbox.x().onTrue( new AlgaePivotPositionCommand( algaePivot, 45.0 ) );
    // mechanicXbox.a().onTrue( new AlgaePivotPositionCommand( algaePivot, 0.0 ) );    

    // climb.setDefaultCommand(
    //     new frc.robot.commands.climb.ClimbDutyCommand(
    //         climb, 
    //         () -> mechanicXbox.getRightY(),
    //         0.5
    //     )
    // );

    // coralIntake.setDefaultCommand(
    //     new frc.robot.commands.coral_intake.CoralIntakeDutyCommand(
    //         coralIntake, 
    //         () -> mechanicXbox.getRightY()
    //     )
    // );

    // coralPivot.setDefaultCommand(
    //     new frc.robot.commands.coral_pivot.CoralPivotDutyCommand(
    //         coralPivot, 
    //         () -> -mechanicXbox.getRightY(),
    //         0.5
    //     )
    // );

    // Intake Position Sets
    // 95 deg, first position ; 265 deg, fully up
    // mechanicXbox.x().onTrue( new CoralPivotPositionCommand( coralPivot, 95.0 ) );
    // mechanicXbox.b().onTrue( new CoralPivotPositionCommand( coralPivot, 265.0 ) );

    // elevator.setDefaultCommand(
    //     new frc.robot.commands.elevator.ElevatorDutyCommand(
    //         elevator, 
    //         () -> -mechanicXbox.getRightY(),
    //         0.1
    //     )
    // );

    // mechanicXbox.x().onTrue( new ElevatorPositionCommand( elevator, 25.4 ) );
    // mechanicXbox.a().onTrue( new ElevatorPositionCommand( elevator, 0.0 ) );  

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Test Auto");
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
