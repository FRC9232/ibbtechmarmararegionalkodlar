// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.commands.ClimbSetCommand;
import frc.robot.commands.IntakeSetCommand;
import frc.robot.commands.ShooterSetCommand;
import frc.robot.commands.swervedrive.drivebase.msDrive;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Rotation2d;

import java.io.File;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.subsystems.LimeVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
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
  IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandJoystick driverController = new CommandJoystick(0);
  final CommandJoystick secondController = new CommandJoystick(1);

  //Autonomous routines
  
  
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    NamedCommands.registerCommand("shooter", new ShooterSetCommand(shooterSubsystem,  "speaker"));
    NamedCommands.registerCommand("intaketoshooter", new IntakeSetCommand(intakeSubsystem, "intakeToShooter"));
    NamedCommands.registerCommand("intake", new IntakeSetCommand(intakeSubsystem,"intake"));
    NamedCommands.registerCommand("autoIntake", new IntakeSetCommand(intakeSubsystem,"autoIntake"));
    NamedCommands.registerCommand("shooterStop", new ShooterSetCommand(shooterSubsystem,"stop"));
    NamedCommands.registerCommand("intakeStop", new IntakeSetCommand(intakeSubsystem,"stop"));
    NamedCommands.registerCommand("intakereversed", new IntakeSetCommand(intakeSubsystem,"outtake"));
    NamedCommands.registerCommand("gyroReset",  Commands.runOnce(drivebase::zeroGyro));
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
    
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_X_DEADBAND),
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -driverController.getRawAxis(2));
    /* 
        
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
    

    m_chooser.setDefaultOption("Top_1_Note", drivebase.getAutonomousCommand("Top_1_Note"));
    m_chooser.addOption("Top_2_Note", drivebase.getAutonomousCommand("Top_2_Note"));
    m_chooser.addOption("Copy of Top_1_Note", drivebase.getAutonomousCommand("Copy of Top_1_Note"));
    m_chooser.addOption("Middle_1_Note", drivebase.getAutonomousCommand("Middle_1_Note"));
    m_chooser.addOption("Middle_2_Note", drivebase.getAutonomousCommand("Middle_2_Note"));
    m_chooser.addOption("Middle_2_Note_Bottom", drivebase.getAutonomousCommand("Middle_2_Note_Bottom"));
    m_chooser.addOption("Middle_3_Note", drivebase.getAutonomousCommand("Middle_3_Note"));
    m_chooser.addOption("Middle_Heading_2_Note", drivebase.getAutonomousCommand("Middle_Heading_2_Note"));
    m_chooser.addOption("Middle_Heading_3_Note", drivebase.getAutonomousCommand("Middle_Heading_3_Note"));
    m_chooser.addOption("yeni_uclu", drivebase.getAutonomousCommand("yeni_uclu"));
    m_chooser.addOption("Bottom_Taksi", drivebase.getAutonomousCommand("Bottom_Taksi"));
    m_chooser.addOption("Bottom_Sigorta", drivebase.getAutonomousCommand("Bottom_Sigorta"));
    m_chooser.addOption("Bottom_1_Note", drivebase.getAutonomousCommand("Bottom_1_Note"));
    m_chooser.addOption("Bottom_2_Note", drivebase.getAutonomousCommand("Bottom_2_Note"));
    m_chooser.addOption("Bottom_1_Far_Note", drivebase.getAutonomousCommand("Bottom_1_Far_Note"));
    m_chooser.addOption("Ust_Deneme", drivebase.getAutonomousCommand("Ust_Deneme"));
    m_chooser.addOption("deneme", drivebase.getAutonomousCommand("deneme"));
    m_chooser.addOption("Only_Subsystem", drivebase.getAutonomousCommand("Only_Subsystem"));
    m_chooser.addOption("SysIDDriveAuto", drivebase.sysIdDriveMotorCommand());
    m_chooser.addOption("SysIDAngleAuto", drivebase.sysIdAngleMotorCommand());
    
    SmartDashboard.putData(m_chooser);
    
    
    SmartDashboard.putData("Run Intake", new IntakeSetCommand(intakeSubsystem, "intake"));
    SmartDashboard.putData("IntakeToShooter", new IntakeSetCommand(intakeSubsystem,  "intakeToShooter"));
    SmartDashboard.putData("HizliIntake", new IntakeSetCommand(intakeSubsystem,  "hizliIntake"));
    SmartDashboard.putData("Run Shooter", new ShooterSetCommand(shooterSubsystem,  "speaker"));
    SmartDashboard.putData("Stop Intake", new IntakeSetCommand(intakeSubsystem,  "stop"));
    SmartDashboard.putData("Stop Shooter", new ShooterSetCommand(shooterSubsystem,  "stop"));

    SmartDashboard.putNumber("Total Current", pdp.getTotalCurrent());

    

     

    // Configure the trigger bindings
    
    
    Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () ->-MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driverController.getRawAxis(2),
        () -> -driverController.getRawAxis(5));
    
    
    
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    configureBindings();
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
    
    Trigger intakeTrigger = driverController.button(6);
    Trigger intakeReversedTrigger = driverController.button(4);
    Trigger intakeToShooterTrigger = driverController.button(8);
    Trigger intakeAndShooterStopTrigger = driverController.button(3);
    
    Trigger shooterSpeakerTrigger = driverController.button(7);
    Trigger shooterAmphTrigger = driverController.button(5);
    
    
    Trigger robotForwardTrigger = driverController.pov(0);
    Trigger robotBackTrigger = driverController.pov(180);
    

    Trigger limeTrigger = driverController.pov(90);
    Trigger robotLeftTrigger = driverController.pov(270);
    
    Trigger shootAreaTrigger= driverController.button(13);

/* 
    limeTrigger.toggleOnTrue(new msDrive(drivebase, limeSubsystem,
        -MathUtil.applyDeadband(driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), 
        -MathUtil.applyDeadband(driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), 
        0.0, 
        true, 
        true));
    */

    limeTrigger.whileTrue(new msDrive(drivebase, limeSubsystem, 0.0, -1.0, 0.0, true, false));
    
    robotForwardTrigger.whileTrue(new msDrive(drivebase, limeSubsystem, 1.0, 0.0, 0.0, true, false));
    robotBackTrigger.whileTrue(new msDrive(drivebase, limeSubsystem, -1.0, 0.0, 0.0, true, false));
    robotLeftTrigger.whileTrue(new msDrive(drivebase, limeSubsystem, 0.0, 1.0, 0.0, true, false));
    
    
    climbTrigger.whileTrue(new ClimbSetCommand(climbSubsystem, 1));
    clibmDownTrigger.whileTrue(new ClimbSetCommand(climbSubsystem, -1));
    climbTrigger.or(clibmDownTrigger).onFalse(new ClimbSetCommand(climbSubsystem, 0));
    
    intakeTrigger.onTrue(new IntakeSetCommand(intakeSubsystem,  "intake"));
    intakeReversedTrigger.onTrue(new IntakeSetCommand(intakeSubsystem, "outtake"));
    intakeToShooterTrigger.onTrue(new IntakeSetCommand(intakeSubsystem,  "intakeToShooter"));
    
    intakeAndShooterStopTrigger.onTrue(new IntakeSetCommand(intakeSubsystem,  "stop"));
    intakeAndShooterStopTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"stop"));

    shooterAmphTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"amph"));
    shooterSpeakerTrigger.onTrue(new ShooterSetCommand(shooterSubsystem,"speaker"));

    Trigger b1Trigger = driverController.button(2);
    b1Trigger.onTrue((Commands.runOnce(drivebase::zeroGyro)));
    Trigger b2Trigger = driverController.button(1);
    b2Trigger.whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

    shootAreaTrigger.whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(1.5, 5.50), Rotation2d.fromDegrees(180)))
                              ));
/* Ikinci Driver Kodları
    Trigger intakeTrigger2 = driverController.button(6);
    Trigger intakeReversedTrigger2 = driverController.button(4);
    Trigger intakeToShooterTrigger2 = driverController.button(8);
    Trigger intakeAndShooterStopTrigger2 = driverController.button(3);
    
    Trigger shooterSpeakerTrigger2 = driverController.button(7);
    Trigger shooterAmphTrigger2 = driverController.button(5);

    intakeTrigger2.onTrue(new IntakeSetCommand(intakeSubsystem,  "intake"));
    intakeReversedTrigger2.onTrue(new IntakeSetCommand(intakeSubsystem, "outtake"));
    intakeToShooterTrigger2.onTrue(new IntakeSetCommand(intakeSubsystem,  "intakeToShooter"));
    
    intakeAndShooterStopTrigger2.onTrue(new IntakeSetCommand(intakeSubsystem,  "stop"));
    intakeAndShooterStopTrigger2.onTrue(new ShooterSetCommand(shooterSubsystem,"stop"));

    shooterAmphTrigger2.onTrue(new ShooterSetCommand(shooterSubsystem,"amph"));
    shooterSpeakerTrigger2.onTrue(new ShooterSetCommand(shooterSubsystem,"speaker"));
    */
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /* 
    driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    */
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  
  }

  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
   

    return m_chooser.getSelected();
    // An example command will be run in autonomous
    //return drivebase.getAutonomousCommand("Top1_Test");
    //return drivebase.sysIdDriveMotorCommand();
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
