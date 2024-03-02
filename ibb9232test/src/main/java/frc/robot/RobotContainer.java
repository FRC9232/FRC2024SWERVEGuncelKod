// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.ClimbSetCommand;
import frc.robot.commands.IntakeSetCommand;
import frc.robot.commands.ShooterSetCommand;
import frc.robot.commands.swervedrive.drivebase.msDrive;

import com.pathplanner.lib.auto.NamedCommands;

import java.io.File;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimeVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ColorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));
  LimeVisionSubsystem limeSubsystem = new LimeVisionSubsystem();
  ColorSubsystem colorSensore = new ColorSubsystem();
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandJoystick driverController = new CommandJoystick(0);
  final CommandJoystick secondController = new CommandJoystick(1);
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
 /* 
    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> -MathUtil.applyDeadband(driverController.getRawAxis(1),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverController.getRawAxis(0),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> -driverController.getRawAxis(2),
                                                                                               
                                                                   driverController.pov(0),
                                                                   driverController.pov(180),
                                                                   driverController.pov(270),
                                                                   driverController.pov(90)); 
  */                                  
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    /* 
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2));
    
        
    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverXbox.getRawAxis(2));
    */
    
    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    NamedCommands.registerCommand("shooter", new ShooterSetCommand(shooterSubsystem,  "speaker"));
    NamedCommands.registerCommand("intaketoshooter", new IntakeSetCommand(intakeSubsystem, colorSensore, "intakeToShooter"));
    NamedCommands.registerCommand("intake", new IntakeSetCommand(intakeSubsystem, colorSensore,"intake"));
    NamedCommands.registerCommand("autoIntake", new IntakeSetCommand(intakeSubsystem, colorSensore,"autoIntake"));
    NamedCommands.registerCommand("shooterStop", new ShooterSetCommand(shooterSubsystem,"stop"));
    NamedCommands.registerCommand("intakeStop", new IntakeSetCommand(intakeSubsystem, colorSensore,"stop"));
    NamedCommands.registerCommand("gyroReset",  Commands.runOnce(drivebase::zeroGyro));



    // Configure the trigger bindings
    configureBindings();

    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () ->-MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2),
        () -> -driverController.getRawAxis(5));
    drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
  
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
    Trigger climbTrigger = driverController.button(9);
    Trigger clibmDownTrigger = driverController.button(10);
    
    Trigger intakeTrigger = secondController.button(6);
    Trigger intakeReversedTrigger = secondController.button(4);
    Trigger intakeToShooterTrigger = secondController.button(8);
    Trigger intakeAndShooterStopTrigger = secondController.button(3);
    
    Trigger shooterSpeakerTrigger = secondController.button(7);
    Trigger shooterAmphTrigger = secondController.button(5);
    
    Trigger robotForwardTrigger = driverController.button(8);
    Trigger robotBackTrigger = driverController.button(7);
    Trigger robotRightTrigger = driverController.pov(90);
    Trigger robotLeftTrigger = driverController.pov(270);

    robotForwardTrigger.whileTrue(new msDrive(drivebase,1.0,0.0,0.0,false));
    robotBackTrigger.whileTrue(new msDrive(drivebase,-1.0,0.0,0.0,false));
    robotRightTrigger.whileTrue(new msDrive(drivebase,0.0,1.0,0.0,false));
    robotLeftTrigger.whileTrue(new msDrive(drivebase,0.0,-1.0,0.0,false));

    climbTrigger.whileTrue(new ClimbSetCommand(climbSubsystem, 1));
    clibmDownTrigger.whileTrue(new ClimbSetCommand(climbSubsystem, -1));
    climbTrigger.or(clibmDownTrigger).onFalse(new ClimbSetCommand(climbSubsystem, 0));
    
    intakeTrigger.onTrue(new IntakeSetCommand(intakeSubsystem, colorSensore,  "intake"));
    intakeReversedTrigger.onTrue(new IntakeSetCommand(intakeSubsystem, colorSensore, "outtake"));
    intakeToShooterTrigger.onTrue(new IntakeSetCommand(intakeSubsystem, colorSensore,  "intakeToShooter"));
    
    intakeAndShooterStopTrigger.onTrue(new IntakeSetCommand(intakeSubsystem, colorSensore,  "stop"));
    intakeAndShooterStopTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"stop"));

    shooterAmphTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"amph"));
    shooterSpeakerTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"speaker"));

    Trigger b1Trigger = driverController.button(2);
    b1Trigger.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    Trigger b2Trigger = driverController.button(1);
    b2Trigger.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
/* 
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("Middle3");
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand(driveFieldOrientedDirectAngle); 
   }


  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
