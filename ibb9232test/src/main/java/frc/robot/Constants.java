// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
  public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
  public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
  
  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.6, 1, 0.8);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
    /* 
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(1.2, 1, 0.4);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(3.5, 1.6, 0.05);
    */
  }

  public static final class DrivebaseConstants
  {

    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.05;
    public static final double LEFT_Y_DEADBAND  = 0.05;
    public static final double RIGHT_X_DEADBAND = 0.05;
    public static final double TURN_CONSTANT    = 6;
  }

  public static class Shooter{
    
    public static final double SHOOTER_VEL = 3000;    
    public static final double SHOOTER_AMPH = 0.22;
    public static final double SHOOTER_SPEAKER = 0.65;

  }

  public static class Intake{
  
    public static final double INTAKE_SPEED = 0.65;    
    public static final double INTAKE_REVERSE_SPEED = -0.7;
    public static final double INTAKE_TO_SHOOTER_SPEED = -0.5;    
    public static final double INTAKE_AUTO_SPEED = 0.60;
    public static final double INTAKE_SHARP_VALUE_LIMIT = 1500;


  }
  public static class Climb {


    
    public static final double CLIMB_SPEED = 0.5;
  
    
  }
}
