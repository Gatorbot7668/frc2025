package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;

public class ShootSubsystem extends SubsystemBase {
  private final CANSparkMax _leadMotor;
  private final CANSparkMax _followMotor;


  public ShootSubsystem() {
    _leadMotor = new CANSparkMax(Constants.SHOOT_MOTOR_PORT, MotorType.kBrushless);
    _followMotor = new CANSparkMax(Constants.SHOOT_FOLLOW_MOTOR_PORT, MotorType.kBrushless);
    
    _leadMotor.restoreFactoryDefaults();
     _followMotor.restoreFactoryDefaults();

    _followMotor.follow(_leadMotor);
  }

  public void shoot() {
    _leadMotor.set(-0.7);
  }

  public void stop() {
    _leadMotor.set(0);
  }

  public Command shootCommand() {
    return runEnd(() -> { shoot(); },
                  () -> { stop();});
  }
}
