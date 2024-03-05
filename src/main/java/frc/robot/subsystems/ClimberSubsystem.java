package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax _motorFollower;
  private final CANSparkMax _motor;

  public ClimberSubsystem() {
    _motor = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_PORT, MotorType.kBrushless);
    _motorFollower = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    _motor.restoreFactoryDefaults();
    _motorFollower.restoreFactoryDefaults();
    _motorFollower.follow(_motor);
  }

  public void up(double speed) {
    _motor.set(speed * 0.5);
  }

  public void stop() {
    _motor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
