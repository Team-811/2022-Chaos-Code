package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimY extends CommandBase {

    private Drivetrain requiredSubsystem;
  
    public LimelightAimY(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
  
    @Override
    public void execute() {
        if (LimelightFetch.getY() > 3) {
        requiredSubsystem.leftWheelsBackward(0.5);
        requiredSubsystem.rightWheelsBackward(0.5);
        }
        else if (LimelightFetch.getY() < -3) {
        requiredSubsystem.leftWheelsForward(0.5);
        requiredSubsystem.rightWheelsForward(0.5);
        }
    }

    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        return true;    
    }
}