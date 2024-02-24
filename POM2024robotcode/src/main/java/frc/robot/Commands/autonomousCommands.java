package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.DriveCommands.DriveMeasured;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeLiftSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingArmSubsystem;
import frc.robot.Subsystems.shooting_subsystems.ShootingSubsystem;

public class autonomousCommands {
    public Command shootTryCollect()
    {
        return arm.closeSlow().alongWith(shoot.spinWheelsCommand()).
            andThen(transfer.transfer(true).raceWith(roller.slow(true))).andThen(shoot.stopWheelsCommand()).
            andThen(arm.OpenForIntakeCommand()).
            andThen(intakeLift.OpenCloseIntake(true)).
            andThen(arm.goToAngleCommand(0).withTimeout(0.4)).
            andThen(arm.goToAngleCommand(0).alongWith(new DriveMeasured(drive, 0.9)).alongWith((roller.intakeNoteCommand()).raceWith(transfer.getFromIntake())));
    }
    public Command shootCollectShoot()
    {
        return shootTryCollect().
        andThen(new DriveMeasured(drive, -0.9).alongWith(shoot.spinWheelsCommand())).alongWith(arm.OpenForIntakeCommand().andThen(intakeLift.OpenCloseIntake(false)).andThen(arm.goToAngleCommand(0))).
        andThen(new WaitCommand(0.17)).
        andThen(transfer.transfer(true)).
        andThen(shoot.stopWheelsCommand().alongWith(transfer.stopWheelsCommand()));        
    }
    public Command shoot()
    {
        return arm.closeSlow().alongWith(shoot.spinWheelsCommand()).
        andThen(new WaitCommand(0.2)).
        andThen(transfer.transfer(true));
    }
    public Command shootMoveOut(){
        return shoot().andThen(new DriveMeasured(drive, 1.5));
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
