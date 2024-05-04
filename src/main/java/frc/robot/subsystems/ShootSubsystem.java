package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
  private final CANSparkMax m_leadMotor;
  private final CANSparkMax m_followMotor;

  public ShootSubsystem() {
    m_leadMotor = new CANSparkMax(Constants.SHOOT_MOTOR_PORT, MotorType.kBrushless);
    m_followMotor = new CANSparkMax(Constants.SHOOT_FOLLOW_MOTOR_PORT, MotorType.kBrushless);
    
    m_leadMotor.restoreFactoryDefaults();
    m_followMotor.restoreFactoryDefaults();

    m_followMotor.follow(m_leadMotor);
  }

  public void shoot() {
    m_leadMotor.set(-0.7);
  }

  public void stop() {
    m_leadMotor.set(0);
  }

  public Command shootCommand() {
    return runEnd(() -> { shoot(); },
                  () -> { stop();});
  }
}
