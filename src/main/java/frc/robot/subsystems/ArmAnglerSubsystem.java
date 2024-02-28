package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmAnglerSubsystem extends SubsystemBase {
  private final PWMSparkMax _ArmAnglerMotorLeft;
  private final PWMSparkMax _ArmAnglerMotorRight;

  public ArmAnglerSubsystem() {
    _ArmAnglerMotorLeft = new PWMSparkMax(Constants.ARMANGLER_MOTOR_LEFT_PORT);
    _ArmAnglerMotorRight = new PWMSparkMax(Constants.ARMANGLER_MOTOR_RIGHT_PORT);
    

  /* 
   * Example command factory method.
   *
   * @return a command
   */
  }

  public void up (double speed) {
    _ArmAnglerMotorLeft.set(speed*0.5);
    _ArmAnglerMotorRight.set(speed*0.5);

  }


  public void stop () {
    _ArmAnglerMotorLeft.set(0);
    _ArmAnglerMotorRight.set(0);
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
