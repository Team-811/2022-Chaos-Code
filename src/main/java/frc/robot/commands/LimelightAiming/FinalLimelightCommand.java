package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class FinalLimelightCommand extends CommandBase{
    private Drivetrain requiredSubsystem;

  
    public FinalLimelightCommand(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
    @Override
    public void initialize() {

    }
  
    @Override
    public void execute() {


    }
    @Override
    public void end(boolean interrupted) {
    }
  
    @Override
    public boolean isFinished() {
        return true;
    }
}
