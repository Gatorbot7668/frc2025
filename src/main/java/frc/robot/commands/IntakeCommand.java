package frc.robot.commands;

import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCommand extends Command {
  private IntakeSubsystem m_Intake;
  private double m_direction;

  public IntakeCommand(IntakeSubsystem Intake, double direction) {
    m_Intake = Intake;
    m_direction = direction;
    addRequirements(m_Intake);
  }

  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.in(m_direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
