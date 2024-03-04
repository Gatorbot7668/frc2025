package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmAnglerSubsystem extends SubsystemBase {
  private final CANSparkMax _ArmAnglerMotorLeft;
  private final CANSparkMax _ArmAnglerMotorRight;
  private Encoder encoder;

  

  public ArmAnglerSubsystem() {
    _ArmAnglerMotorLeft = new CANSparkMax(Constants.ARMANGLER_MOTOR_LEFT_PORT, MotorType.kBrushless);
    _ArmAnglerMotorRight = new CANSparkMax(Constants.ARMANGLER_MOTOR_RIGHT_PORT, MotorType.kBrushless);
    _ArmAnglerMotorLeft.restoreFactoryDefaults();
    _ArmAnglerMotorRight.restoreFactoryDefaults();

    _ArmAnglerMotorLeft.follow(_ArmAnglerMotorRight, true);
    //_ArmAnglerMotorRight.setInverted(true);
    encoder = new Encoder(Constants.ArmConstants.kEncoderPorts[0],
                          Constants.ArmConstants.kEncoderPorts[1]);

  

  /* 
   * Example command factory method.
   *
   * @return a command
   */
  }

  public void up (double speed) {
    _ArmAnglerMotorRight.set(speed*0.4);
    

  }


  public void stop () {
    _ArmAnglerMotorRight.set(0);
   
  }

  public void move(DoubleSupplier s) {
    _ArmAnglerMotorRight.set(s.getAsDouble());
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm/encoder", encoder.getDistance());
    SmartDashboard.putNumber("arm/motor_right", _ArmAnglerMotorRight.get());
    SmartDashboard.putNumber("arm/motor_left", _ArmAnglerMotorLeft.get());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
