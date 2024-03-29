// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.swervedrive.drivebase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeVisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
/**
 * An example command that uses an example subsystem.
 */
public class msDrive extends Command
{

  private final SwerveSubsystem  swerve;
  private final LimeVisionSubsystem limeSubsystem;
  private final Double   vX;
  private final Double   vY;
  private final Double  omega;
  private final Boolean  fieldRelative;
  private final Boolean lime;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The subsystem used by this command.
   */
  public msDrive(SwerveSubsystem swerve,LimeVisionSubsystem limeSubsystem, Double vX, Double vY, Double omega, Boolean fieldRelative, Boolean lime)
  {
    this.swerve = swerve;
    this.limeSubsystem = limeSubsystem;
    this.vX = vX;
    this.vY = vY;
    this.omega = omega;
    this.lime = lime;
    this.fieldRelative = fieldRelative;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve,limeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    if (lime){
      limeSubsystem.setPipeline(2);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    double xVelocity;
    double yVelocity;
    double angVelocity;
    if(lime){
      angVelocity = limeSubsystem.getAimOmega();
      xVelocity = Math.pow(vX, 3);
      yVelocity = Math.pow(vY, 3);
    }
    else {
      angVelocity = omega;
      xVelocity   = vX;
      yVelocity = vY;
    }
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
