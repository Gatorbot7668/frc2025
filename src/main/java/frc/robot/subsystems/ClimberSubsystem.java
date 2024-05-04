package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMax m_motorFollower;
  private final CANSparkMax m_motor;

  public ClimberSubsystem() {
    m_motor = new CANSparkMax(Constants.CLIMBER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    m_motorFollower = new CANSparkMax(Constants.CLIMBER_MOTOR_LEFT_PORT, MotorType.kBrushless);
   
    m_motor.restoreFactoryDefaults();
    m_motorFollower.restoreFactoryDefaults();
    
  //  _motorFollower.follow(_motor);
  }

  public void up(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  public Command climbCommand(double speed) {
    return runEnd(() -> { up(speed); },
                  () -> { stop();});
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
