package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.DriveCommands.DriveMeasured;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeLiftSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingArmSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingSubsystem;

public class autonomousCommands {
    public Command shootMoveOut()
    {
        return arm.OpenForIntakeCommand().
            andThen(intakeLift.OpenCloseIntake(true)).
            andThen(arm.closeSlow().alongWith(shoot.spinWheelsCommand())).
            andThen(transfer.transfer(true)).
            andThen(shoot.stopWheelsCommand().alongWith(transfer.stopWheelsCommand())).
            andThen(new DriveMeasured(drive, 0.9).alongWith((roller.intakeNoteCommand()).raceWith(transfer.getFromIntake())));
    }
    public Command shootCollectShoot()
    {
        return shootMoveOut().
        andThen(new DriveMeasured(drive, -0.9).alongWith(shoot.spinWheelsCommand())).
        andThen(transfer.transfer(true)).
        andThen(shoot.stopWheelsCommand().alongWith(transfer.stopWheelsCommand()));        
    }
    public Command shoot()
    {
        return arm.closeSlow().alongWith(shoot.spinWheelsCommand()).
        andThen(transfer.transfer(true));
    }
    public autonomousCommands(DriveSubsystem drive, IntakeLiftSubsystem intakeLift, IntakeRollerSubsystem roller, ShootingSubsystem shoot, ShootingArmSubsystem arm, TransferSubsystem transfer)
    {
        this.drive = drive;
        this.intakeLift = intakeLift;
        this.roller = roller;
        this.shoot = shoot;
        this.arm = arm;
        this.transfer = transfer;
    }

    DriveSubsystem drive;
    IntakeLiftSubsystem intakeLift;
    IntakeRollerSubsystem roller;
    ShootingSubsystem shoot;
    ShootingArmSubsystem arm;
    TransferSubsystem transfer;

}
