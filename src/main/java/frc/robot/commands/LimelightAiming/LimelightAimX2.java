package frc.robot.commands.LimelightAiming;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Vision.LimelightFetch;
import frc.robot.subsystems.Drivetrain;

public class LimelightAimX2 extends CommandBase{

    private Drivetrain requiredSubsystem;
    private double left_command;
    private double right_command;
  
    public LimelightAimX2(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
    }
    @Override
    public void initialize() {

    }
  
    @Override
    public void execute() {
        System.out.println("AimX2 is running");
        double tx = LimelightFetch.getX();
        float Kp = 0.02f; //turn speed constant
        float min_command = 0.10f;
        float heading_error = (float)tx;
        float steering_adjust = 0.0f;
        left_command = 0;
        right_command = 0;
        if(tx > 1.0){
            steering_adjust = Kp*heading_error - min_command;
        }
        if (tx < 1.0){
            steering_adjust = Kp*heading_error + min_command;
        }
        left_command += steering_adjust;
        right_command -= steering_adjust;
        requiredSubsystem.leftWheelsForward(left_command);
        requiredSubsystem.rightWheelsForward(right_command);

    }
    @Override
    public void end(boolean interrupted) {
        requiredSubsystem.stopRobot();
    }
  
    @Override
    public boolean isFinished() {
        double x = LimelightFetch.getX();
        if(x >= -4.5 && x <= 4.5 && x !=0.0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}
