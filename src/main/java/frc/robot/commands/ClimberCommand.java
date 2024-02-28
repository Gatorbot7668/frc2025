package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberCommand extends Command {
     private ClimberSubsystem m_Climber;
     private double m_direction;

   public ClimberCommand(ClimberSubsystem Climber, float direction) {
    m_Climber = Climber;
    m_direction = direction;
    addRequirements(m_Climber);

   }


  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_Climber.up(m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Climber.stop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
  }


}
