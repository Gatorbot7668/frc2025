package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class ArmAnglerSubsystem extends SubsystemBase {
  private final CANSparkMax _motorFollower;
  private final CANSparkMax _motor;
  private Encoder encoder;
  private RelativeEncoder neoEncoder;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine;

  public ArmAnglerSubsystem() {
    _motor = new CANSparkMax(Constants.ARMANGLER_MOTOR_LEFT_PORT, MotorType.kBrushless);
    _motorFollower = new CANSparkMax(Constants.ARMANGLER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motorFollower.restoreFactoryDefaults();

    _motorFollower.follow(_motor, true);
    encoder = new Encoder(Constants.ArmConstants.kEncoderPorts[0],
        Constants.ArmConstants.kEncoderPorts[1]);
    neoEncoder = _motor.getEncoder();

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
                        m_appliedVoltage.mut_replace(
                            _motor.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(encoder.getDistance(), Degrees))
                    .angularVelocity(
                        m_velocity.mut_replace(encoder.getRate(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));        
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
    SmartDashboard.putNumber("arm/encoder", encoder.getDistance());
    SmartDashboard.putNumber("arm/motor", _motor.get());
    SmartDashboard.putNumber("arm/position", neoEncoder.getPosition());
    SmartDashboard.putNumber("arm/velocity", neoEncoder.getVelocity());
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
