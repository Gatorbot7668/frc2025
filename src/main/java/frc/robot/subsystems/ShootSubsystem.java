package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.util.CANSparkMaxSendable;

public class ShootSubsystem extends SubsystemBase {
  private final CANSparkMaxSendable m_motor;
  private final CANSparkMaxSendable m_followMotor;

  public ShootSubsystem() {
    m_motor = new CANSparkMaxSendable(Constants.kShootMotorPorts.port1(), MotorType.kBrushless);
    m_followMotor = new CANSparkMaxSendable(Constants.kShootMotorPorts.port2(), MotorType.kBrushless);
    
    m_motor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_followMotor.follow(m_motor);

    addChild("motor", m_motor);
  }

  public void shoot(double speed) {
    m_motor.set(speed);
  }

  public void stop() {
    m_motor.set(0);
  }

  public Command shootCommand(double speed) {
    return runEnd(() -> { shoot(speed); },
                  () -> { stop();});
  }
}
