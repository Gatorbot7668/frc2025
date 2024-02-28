package frc.robot.commands;

import frc.robot.subsystems.ArmAnglerSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmAnglerCommand extends Command {
   private ArmAnglerSubsystem m_armAngler;
   private double m_direction;

   public ArmAnglerCommand(ArmAnglerSubsystem ArmAngler, float direction) {
    m_armAngler = ArmAngler;
    m_direction = direction;
    addRequirements(m_armAngler);

   }


  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_armAngler.up(m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armAngler.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }

}
