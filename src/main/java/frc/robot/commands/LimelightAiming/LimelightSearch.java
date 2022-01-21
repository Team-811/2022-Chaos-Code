package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightSearch extends CommandBase{
    
    private Drivetrain requiredSubsystem;

    public LimelightSearch(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);      
    }
    @Override
    public void initialize() {System.out.println("Seaching");
    }
    @Override
    public void execute() {
        
        requiredSubsystem.turnLeft(Constants.AIM_SPEED);


    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        double inVision = LimelightFetch.getV();
   
        if(inVision>0)
            {
                return true;
            }
            else
            {
                return false;
            }
    }
}
