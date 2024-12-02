// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CANDeviceID;
import frc.robot.util.CANSparkMaxSendable;

public class ClawSubsystem extends SubsystemBase {
  private final CANSparkMaxSendable m_motor;
  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    m_motor = new CANSparkMaxSendable(CANDeviceID.kClawMotor, MotorType.kBrushless);
  }

  public void move (double speed) {
    m_motor.set(speed);
  }

  public void stop (){
    m_motor.set(0);
  }

  public Command clawCommand (double speed){
    return runEnd(() -> { move(speed); },
                  () -> { stop();});
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
