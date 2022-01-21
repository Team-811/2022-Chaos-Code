package frc.robot.commands.Drivetrain;


import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.controllers.BobXboxController;
import frc.robot.subsystems.Drivetrain;

public class DrivingCommand extends CommandBase {
  
  private Drivetrain requiredSubsystem;

   static BobXboxController controller;
  
  public DrivingCommand(Drivetrain m_SubsystemBase, BobXboxController m_controller) {
    requiredSubsystem = m_SubsystemBase;
    controller = m_controller;
    addRequirements(requiredSubsystem);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {

    // if(RobotContainer.driveController.rightStick.getX() == 0.0){
      
    //   requiredSubsystem.driveWithMisery(-RobotContainer.driveController.leftStick.getY(),
    //   RobotContainer.driveController.rightStick.getY(), requiredSubsystem.turnToAngle(requiredSubsystem.getGyro().getAngle()));
    // }else {
    requiredSubsystem.driveWithMisery(-RobotContainer.driveController.leftStick.getY(),
        RobotContainer.driveController.rightStick.getY(), RobotContainer.driveController.rightStick.getX());
   // }
      }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
