package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

// This mechanism uses
//   NEO v1.1 brushless motor: https://www.revrobotics.com/rev-21-1650/ 
//   REV Through Bore Encoder: https://www.revrobotics.com/rev-11-1271/
//   Spark Max motor controller: https://www.revrobotics.com/rev-11-2158/

// Arm theory (combine feeedback and feedforward controls):
// https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
//
// Is it the case that Trapezoidal motion profile is needed because feedforward
// can only be calculated for a given angle, so need fine-grained progression of setpoints?
// The article above hints at that, in
// "accurately converge to the setpoint over time after a “jump” command"

// Switching to spark's internal PID calculator might be better
//  for reasons explained in https://www.chiefdelphi.com/t/spark-max-pid/340527/7 
//  more precise control and  
// https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control
// and then we'd switch to ProfiledPIDSubsystem to TrapezoidProfileSubsystem
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbotoffboard/subsystems/ArmSubsystem.java
// and https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Position%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
// Will need to configure setFeedbackDevice like it is done here
// https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Encoder%20Feedback%20Device/src/main/java/frc/robot/Robot.java
//
// Hardware setup for it: https://docs.revrobotics.com/sparkmax/operating-modes/using-encoders

// Sysid code is taken from
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/sysid/subsystems/Shooter.java
// ProfiledPIDSubsysem implementation is from
// https://github.com/wpilibsuite/allwpilib/blob/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples/armbot/subsystems/ArmSubsystem.java

// Don't use Rev's SmartMotion, explained in https://www.chiefdelphi.com/t/understanding-and-tuning-smart-motion-for-an-arm/426639/5
// and acknowledged by Rev in a reply to the post.

// All angular quantities without units are in radians.

// Note on the method of motor control:
//   SparkMax has different control modes (kCtrlType in https://docs.revrobotics.com/sparkmax/software-resources/configuration-parameters)
//   and they cannot be mixed. Set/get is Duty Cycle and set/getVoltage is Voltage. Explanation here
//     https://www.chiefdelphi.com/t/sparkmax-set-vs-setvoltage/415059
//   set/getVoltage should be better for us because we need absolute power control when
//   trying to move to an angle in presence of gravitational force. Also
//   https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/combining-feedforward-feedback.html#using-feedforward-components-with-pid
//      Since feedforward voltages are physically meaningful, it is best to use the setVoltage()
//      method when applying them to motors to compensate for “voltage sag” from the battery."
//
//   (Velocity and Position are two other control modes and these seem to be not useful for a
//   mechanism whose feedforward component is identified using SysId, which is all about voltage)

public class ArmAnglerSubsystem extends ProfiledPIDSubsystem {
  private final CANSparkMax _motorFollower;
  private final CANSparkMax _motor;
  private DutyCycleEncoder absEncoder;
  private Encoder relEncoder;
  private RelativeEncoder neoEncoder;
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine;

  public ArmAnglerSubsystem() {
    super(
        new ProfiledPIDController(
            ArmConstants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                ArmConstants.kMaxVelocityRadPerSecond,
                ArmConstants.kMaxAccelerationRadPerSecSquared)));
    // Start pointing up
    setGoal(Units.degreesToRadians(90));

    _motor = new CANSparkMax(Constants.ARMANGLER_MOTOR_LEFT_PORT, MotorType.kBrushless);
    _motorFollower = new CANSparkMax(Constants.ARMANGLER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motorFollower.restoreFactoryDefaults();

    _motorFollower.follow(_motor, true);
    absEncoder = new DutyCycleEncoder(Constants.ArmConstants.kDutyEncoderPort);
    // This returns the internal hall sensor whose counts per revolution is 42, very low.
    // Probably not useful because the resolution is so poor.
    neoEncoder = _motor.getEncoder();
    relEncoder = new Encoder(Constants.ArmConstants.kEncoderPorts[0],
                              Constants.ArmConstants.kEncoderPorts[1]);

    // See the explanation of CPR and PPR at
    // https://docs.wpilib.org/en/stable/docs/hardware/sensors/encoders-hardware.html#quaderature-encoder-resolution
    // For our Through Bore Encoder, Cycles per Revolution is 2048 per http://revrobotics.com/rev-11-1271/
    final int kRevThoroughBoreEncoderPPR = 2048;
    // 1 / PPR is how many rotations per pulse
    relEncoder.setDistancePerPulse(Units.rotationsToRadians(1.0 / kRevThoroughBoreEncoderPPR));
    // Similar, set absolute encoder units to be in radians
    absEncoder.setDistancePerRotation(Units.rotationsToRadians(1));

    m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              (Measure<Voltage> volts) -> {
                _motor.setVoltage(volts.in(Volts));
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("arm")
                    .voltage(
                        // Annoingly there is no getVoltage() method and get() cannot be used with setVoltage();
                        // see the discussion at
                        // https://www.chiefdelphi.com/t/sysid-routine-not-properly-recording-motor-speed/455172
                        m_appliedVoltage.mut_replace(
                            // _motor.get() * RobotController.getBatteryVoltage(),
                            _motor.getBusVoltage() * _motor.getAppliedOutput(),
                            Volts))
                    .angularPosition(m_angle.mut_replace(getMeasurement(), Radians))
                    .angularVelocity(
                        m_velocity.mut_replace(relEncoder.getRate(), RadiansPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));        
  }
  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the setpoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    _motor.setVoltage(output + feedforward);
  }

  @Override
  // Returns radians
  public double getMeasurement() {
    return absEncoder.getDistance() + ArmConstants.kArmOffsetRadians;
  }
  /*
   * Example command factory method.
   *
   * @return a command
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  public void stop() {
    _motor.set(0);
  }

  public void move(DoubleSupplier s) {
    _motor.set(s.getAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    super.periodic();
    SmartDashboard.putNumber("arm/encoder", absEncoder.getDistance());
    SmartDashboard.putNumber("arm/adjust_encoder", getMeasurement());
    SmartDashboard.putNumber("arm/motor", _motor.get());
    SmartDashboard.putNumber("arm/motor_bus_volt", _motor.getBusVoltage());
    SmartDashboard.putNumber("arm/motor_applied_output", _motor.getAppliedOutput());
    SmartDashboard.putNumber("arm/motor_bus_comp", _motor.getVoltageCompensationNominalVoltage());
    SmartDashboard.putNumber("arm/rate", relEncoder.getRate());
    SmartDashboard.putNumber("arm/neorate", neoEncoder.getVelocity());
    SmartDashboard.putNumber("arm/bat_voltage", RobotController.getBatteryVoltage());
  }

  public Command moveArm(DoubleSupplier speed) {
    return runEnd(() -> { move(speed); },
                  () -> { stop();});
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
