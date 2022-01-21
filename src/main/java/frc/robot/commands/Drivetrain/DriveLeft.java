package frc.robot.commands.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class DriveLeft extends CommandBase {
    private Drivetrain requiredSubsystem;
    private double FL;
    private double FR;
    private double BL;
    private double BR;
  public DriveLeft(Drivetrain m_SubsystemBase) {
    requiredSubsystem = m_SubsystemBase;
    addRequirements(requiredSubsystem);

  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() { 
       FL = -0.7;
       FR = -0.7;
       BL = 0.7;
       BR = 0.7;

       if(RobotContainer.driveController.rightStick.getX() == 0.0){      
        requiredSubsystem.driveWithMisery(-RobotContainer.driveController.leftStick.getY(),
        RobotContainer.driveController.rightStick.getY(), requiredSubsystem.turnToAngle(requiredSubsystem.getGyro().getAngle()), FL, FR, BL, BR);
       }
       else{
       requiredSubsystem.driveWithMisery(-RobotContainer.driveController.leftStick.getY(),
       RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX(), FL, FR, BL, BR);
       }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return true;
  }

}
