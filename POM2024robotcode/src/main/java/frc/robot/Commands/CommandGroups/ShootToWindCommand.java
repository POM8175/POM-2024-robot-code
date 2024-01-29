package frc.robot.Commands.CommandGroups;

import static frc.robot.Constants.ShootingConstants.SHOOT_TO_WING_SPEED;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Subsystems.shooting_subsystems.ShootingArmSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingSubsystem;

public class ShootToWindCommand extends ParallelCommandGroup{
    public ShootToWindCommand(ShootingArmSubsystem shootingArmSubsystem,ShootingSubsystem shooting_Subsystem ){ {

    addRequirements(shootingArmSubsystem,shooting_Subsystem);
    addCommands(

        new ShootingSubsystem().spinWheelsToSpeedCommand(SHOOT_TO_WING_SPEED)
        //  add arm rotation here 

    );       
        
    }
    
}
}