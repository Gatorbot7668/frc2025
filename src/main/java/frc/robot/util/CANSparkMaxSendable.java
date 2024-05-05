package frc.robot.util;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class CANSparkMaxSendable extends CANSparkMax implements Sendable {
  private CANSparkMaxSendableAdapter m_sendableAdapter;

  public CANSparkMaxSendable(int deviceId, MotorType type) {
    super(deviceId, type);
    m_sendableAdapter = new CANSparkMaxSendableAdapter(this);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    m_sendableAdapter.initSendable(builder);
  }  
}
