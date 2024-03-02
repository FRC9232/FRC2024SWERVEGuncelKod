// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
/**
 * An example command that uses an example subsystem.
 */
public class msDrive extends Command
{

  private final SwerveSubsystem  swerve;
  private final Double   vX;
  private final Double   vY;
  private final Double  omega;
  private final Boolean  fieldRelative;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public msDrive(SwerveSubsystem swerve, Double vX, Double vY, Double omega, Boolean fieldRelative)
  {
    this.swerve = swerve;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double xVelocity   = vX;
    double yVelocity   = vY;
    double angVelocity = omega;
    SmartDashboard.putNumber("vX_ms", xVelocity);
    SmartDashboard.putNumber("vY_ms", yVelocity);
    SmartDashboard.putNumber("omega_ms", angVelocity);
    swerve.drive(new Translation2d(xVelocity,yVelocity), angVelocity, fieldRelative);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
