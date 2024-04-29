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

  //port constants
  public static final int INTAKE_MOTOR_PORT = 15; 
  public static final int SHOOT_MOTOR_PORT = 14; 
  public static final int ARMANGLER_MOTOR_LEFT_PORT = 16; 
  public static final int ARMANGLER_MOTOR_RIGHT_PORT = 17; 
  public static final int CLIMBER_MOTOR_LEFT_PORT = 18; 
  public static final int CLIMBER_MOTOR_RIGHT_PORT = 19; 
  public static final int SHOOT_FOLLOW_MOTOR_PORT = 13;
  

  public static final class ArmConstants {
    public static final double kSVolts = 1;
    public static final int[] kEncoderPorts = new int[]{1,2};
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0;
    public static final double kP = 1;
    public static final double kMaxVelocityRadPerSecond = 3;
    public static final double kMaxAccelerationRadPerSecSquared = 10;
    // TODO: check units and describe how this was measured.
    // 0.25 is quarter of a rotation (looking up, or 90 degrees)
    // 0.262274 is offset that we measured when balancing the arm at 90 degrees,
    // but what was the unit? Record what encoder we used to observe it.
    // if absEncoder.getDistance() then the units were correct, but not any more
    // because we now programmed it to be in radians (with setDistancePerRotation )
    public static final double kArmOffsetRadians = Units.rotationsToRadians(0.262274-0.25);
    public static final int kDutyEncoderPort = 0;
  }
  
  public static final class AutonConstants
  {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants ANGLE_PID   = new PIDConstants(0.4, 0, 0.01);
  }

  public static final class DrivebaseConstants
  {
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10; // seconds
  }

  public static class OperatorConstants
  {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryDriverControllerPort = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
    
    public static final double TURN_CONSTANT    = 6;
  }

  
  // Whether TunableNumbers are changeable via SmartDashboard
  public static boolean tuningMode = true;
}