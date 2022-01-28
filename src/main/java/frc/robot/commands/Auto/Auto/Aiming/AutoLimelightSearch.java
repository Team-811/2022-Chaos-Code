package frc.robot.commands.Auto.Auto.Aiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class AutoLimelightSearch extends CommandBase{
    
    private Drivetrain requiredSubsystem;
    private boolean found = false;

    public AutoLimelightSearch(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);      
    }
    
    @Override
    public void execute() {
        double seen = LimelightFetch.getV();

        if (seen == 1.0)
            found = true;
        else
            requiredSubsystem.turnLeft(Constants.AIM_SPEED);

    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        if (found == true)
            {
                found = false;
                return true;
            }
            else
            {
                return false;
            }
    }
}
