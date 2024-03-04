package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;


public class ShootSubsystem extends SubsystemBase {
   private final CANSparkMax _ShootMotor;
   

    public ShootSubsystem () {
        _ShootMotor = new CANSparkMax(Constants.SHOOT_MOTOR_PORT, MotorType.kBrushless);
    }

    public void shoot () {
        _ShootMotor.set(0.5);
       
    
      }
    
    
      public void stop () {
        _ShootMotor.set(0);
        
      }
    
}
