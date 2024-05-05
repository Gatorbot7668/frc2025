package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.CANSparkMaxSendable;

public class ClimberSubsystem extends SubsystemBase {
  private final CANSparkMaxSendable m_motorFollower;
  private final CANSparkMaxSendable m_motor;

  public ClimberSubsystem() {
    m_motor = new CANSparkMaxSendable(Constants.CLIMBER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    m_motorFollower = new CANSparkMaxSendable(Constants.CLIMBER_MOTOR_LEFT_PORT, MotorType.kBrushless);
   
    m_motor.restoreFactoryDefaults();
    m_motorFollower.restoreFactoryDefaults();
    
    m_motorFollower.follow(m_motor);

    addChild("motor", m_motor);
  }

  public void climb(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  public Command climbCommand(double speed) {
    return runEnd(() -> { climb(speed); },
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
