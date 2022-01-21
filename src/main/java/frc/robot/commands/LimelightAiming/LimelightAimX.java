package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimX extends CommandBase{

    private Drivetrain requiredSubsystem;
  
    public LimelightAimX(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      System.out.println("Seachdas");
    }
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
        double x = LimelightFetch.getX();
        System.out.print("Bruh");
        if(x < -1){
        requiredSubsystem.turnLeft(0.06);
        }
        if (x > 1){
        requiredSubsystem.turnRight(0.06);
    }
    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        double x = LimelightFetch.getX();
        if(x >= -1 && x <= 1 && x !=0.0)
        {
            return false;
        }
        else
        {
            return true;
        }
    }
}
