// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Per;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import swervelib.math.Matter;

public final class Constants
{
  public static record TwoIDs(int id1, int id2) {}

 
  public static final class CANDeviceID {
    // CAN IDs 1-8 are taken by Swerve motors and 9-12 - by Swerve encoders

    public static final int kClawMotor = 10000;

  }

  public static final class DIOPort {
    public static final int kDutyEncoder = 0;
    public static final int kQuadratureEncoderChannelA = 1;
    public static final int kQuadratureEncoderChannelB = 2;
  }

  public static class SwerveConstants {
    // see also PID constants in pidfproperties.json

    // Maximum speed of the robot in meters per second, used to limit acceleration.
    public final static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
  }

  public static final class PathPlannerConstants {
    public static final PIDConstants kTranslationPID = new PIDConstants(0.7, 0, 0);
    public static final PIDConstants kAnglePID = new PIDConstants(0.4, 0, 0.01);
    public final static Measure<Velocity<Distance>> kMaxSpeed = MetersPerSecond.of(4);
  }

  public static final class DrivebaseConstants {
    // Hold time on motor brakes when disabled
    public static final Measure<Time> kWheelLockTime = Seconds.of(10);
  }
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kSecondaryDriverControllerPort = 1;

    // Joystick Deadband
    public static final double LEFT_X_DEADBAND  = 0.1;
    public static final double LEFT_Y_DEADBAND  = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double RIGHT_Y_DEADBAND = 0.1;
  }

  public static class AdvancedDriveCommandsConstants {
    public static final Measure<Mass> kRobotMass = Pounds.of(50);
    public static final Matter kChassisMatter = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), kRobotMass.in(Kilograms));
    public static final Measure<Time> kLooptime = Milliseconds.of(130); // 20ms + 110ms sprk max velocity lag
    public static final Measure<Velocity<Angle>> kTurnSpeed = RotationsPerSecond.of(1);
  }

  // Whether TunableNumbers are changeable via SmartDashboard
  public static final boolean kTuningMode = true;
}
