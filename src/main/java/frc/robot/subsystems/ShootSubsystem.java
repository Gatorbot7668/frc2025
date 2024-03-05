package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
  private final CANSparkMax _motor;

  public ShootSubsystem() {
    _motor = new CANSparkMax(Constants.SHOOT_MOTOR_PORT, MotorType.kBrushless);
  }

  public void shoot() {
    _motor.set(0.5);
  }

  public void stop() {
    _motor.set(0);
  }
}
