package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final PWMSparkMax _ClimberMotorLeft;
  private final PWMSparkMax _ClimberMotorRight;

  public ClimberSubsystem() {
    _ClimberMotorLeft = new PWMSparkMax(Constants.CLIMBER_MOTOR_LEFT_PORT);
    _ClimberMotorRight = new PWMSparkMax(Constants.CLIMBER_MOTOR_RIGHT_PORT);
    

  /* 
   * Example command factory method.
   *
   * @return a command
   */
  }

  public void up (double speed) {
    _ClimberMotorLeft.set(speed*0.5);
    _ClimberMotorRight.set(speed*0.5);

  }


  public void stop () {
    _ClimberMotorLeft.set(0);
    _ClimberMotorRight.set(0);
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
