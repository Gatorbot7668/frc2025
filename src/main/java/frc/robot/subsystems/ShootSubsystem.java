package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ShootSubsystem extends SubsystemBase {
   private final PWMSparkMax _ShootMotorLeft;
   private final PWMSparkMax _ShootMotorRight;

    public ShootSubsystem () {
        _ShootMotorLeft = new PWMSparkMax(Constants.SHOOT_MOTOR_LEFT_PORT);
        _ShootMotorRight = new PWMSparkMax(Constants.SHOOT_MOTOR_RIGHT_PORT);
    }

    public void shoot () {
        _ShootMotorLeft.set(0.5);
        _ShootMotorRight.set(0.5);
    
      }
    
    
      public void stop () {
        _ShootMotorLeft.set(0);
        _ShootMotorRight.set(0);
      }
    
}
