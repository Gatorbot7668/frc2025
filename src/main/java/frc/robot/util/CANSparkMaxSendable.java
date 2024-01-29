package frc.robot.util;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class CANSparkMaxSendable implements Sendable  {
  private CANSparkMax m_motor;

  public CANSparkMaxSendable(CANSparkMax motor) {
    m_motor = motor;
    SendableRegistry.addLW(this, "SparkMax", motor.getDeviceId());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Motor Controller");
    builder.setActuator(true);
    builder.setSafeState(m_motor::stopMotor);
    builder.addDoubleProperty("Value", m_motor::get, m_motor::set);
  }
}
