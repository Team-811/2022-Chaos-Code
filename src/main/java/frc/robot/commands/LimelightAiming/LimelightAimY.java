package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimY extends CommandBase {
    
    private Drivetrain requiredSubsystem;
    private double x = LimelightFetch.getX();
  
    public LimelightAimY(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        if(x < 17){
        requiredSubsystem.driveForward(Constants.AIM_SPEED);
    }
    if (x > 19){
        requiredSubsystem.driveBackwards(Constants.AIM_SPEED);
    }
    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
                double inVision = LimelightFetch.getV();
        if(inVision<0.5)
            return true;
        
        if(x >= 20 && x <= 22)
        {
            return true;
        }
        else
        {
            return false;
        }

    }
    
}
