// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final PWMSparkMax _IntakeMotorLeft;
  private final PWMSparkMax _IntakeMotorRight;

  public IntakeSubsystem() {
    _IntakeMotorLeft = new PWMSparkMax(Constants.INTAKE_MOTOR_LEFT_PORT);
    _IntakeMotorRight = new PWMSparkMax(Constants.INTAKE_MOTOR_RIGHT_PORT);
    

  /* 
   * Example command factory method.
   *
   * @return a command
   */
  }

  public void in () {
    _IntakeMotorLeft.set(0.5);
    _IntakeMotorRight.set(0.5);

  }


  public void stop () {
    _IntakeMotorLeft.set(0);
    _IntakeMotorRight.set(0);
  }

  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
