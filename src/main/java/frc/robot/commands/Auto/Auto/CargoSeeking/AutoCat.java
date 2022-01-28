package frc.robot.commands.Auto.Auto.CargoSeeking;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

public class AutoCat extends SequentialCommandGroup{
    private Drivetrain requiredSubsystem;
    
    public AutoCat(Drivetrain m_SubsystemBase) {
      requiredSubsystem = m_SubsystemBase;
      addRequirements(requiredSubsystem);
      addCommands(new AutoCatSearch(requiredSubsystem), new AutoCatFollow(requiredSubsystem));
    }
}
