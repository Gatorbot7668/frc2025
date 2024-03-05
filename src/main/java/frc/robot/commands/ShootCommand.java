package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShootSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootCommand extends Command {
  private ShootSubsystem m_Shoot;

  public ShootCommand(ShootSubsystem Shoot) {
    m_Shoot = Shoot;
    addRequirements(m_Shoot);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Shoot.shoot();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Shoot.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
