package frc.robot.commands;

import frc.robot.subsystems.ArmAnglerSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ArmAnglerCommand extends Command {
   private ArmAnglerSubsystem m_armAngler;
   private DoubleSupplier m_speedSupplier;

   public ArmAnglerCommand(ArmAnglerSubsystem ArmAngler, DoubleSupplier speedSupplier) {
    m_armAngler = ArmAngler;
    m_speedSupplier = speedSupplier;
    addRequirements(m_armAngler);
   }


  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      m_armAngler.move(m_speedSupplier);
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
