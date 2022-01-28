package frc.robot.commands.Auto.Auto.Aiming;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutoLimelightAim extends SequentialCommandGroup{
    private Drivetrain requiredSubsystem;
    
    public AutoLimelightAim(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(/*new LimelightSearch(requiredSubsystem), */new AutoLimelightAimX(requiredSubsystem));
    }
}
