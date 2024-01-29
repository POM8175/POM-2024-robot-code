package frc.robot.Commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingSubsystem;

public class SpinWheelsThenShoot extends SequentialCommandGroup{

    public SpinWheelsThenShoot(ShootingSubsystem shootingSubsystem, IntakeRollerSubsystem intakeRollerSubsystem){
        addRequirements(shootingSubsystem, intakeRollerSubsystem);
        addCommands(
            new ShootingSubsystem().spinWheelsCommand()
            //add intake here

        );
    }
    
    
}
