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
    // public Command shootTryCollect()
    // {
    //     return arm.closeSlow().alongWith(shoot.spinWheelsCommand()).
    //         andThen(new WaitCommand(0.9)).
    //         andThen(transfer.transfer(true).raceWith(roller.slow(true))).andThen(shoot.stopWheelsCommand()).
    //         andThen(arm.OpenForIntakeCommand()).
    //         andThen(intakeLift.OpenCloseIntake(true)).
    //         andThen(arm.goToAngleCommand(0)).
    //         andThen(arm.goToAngleCommand(0).alongWith(new DriveMeasured(drive, 0.9)).alongWith((roller.intakeNoteCommand()).raceWith(transfer.getFromIntake())));
    // }
    public Command shootTryCollect()
    {
        return arm.OpenForIntakeCommand().
            andThen(intakeLift.OpenCloseIntakeTimersWithNote(true)).
            andThen(arm.closeSlow().alongWith(shoot.spinWheelsCommand())).
            andThen(new WaitCommand(0.5)).
            andThen(roller.intakeNoteCommand().raceWith(transfer.inForShootCommand())).
            andThen(transfer.transfer(true).raceWith(roller.slow(true))).
            andThen(shoot.stopWheelsCommand()).
            andThen(new DriveMeasured(drive, 1).alongWith((roller.intakeNoteCommand()).raceWith(transfer.getFromIntake())));
    }
    public Command shootCollectShoot()
    {
        return shootTryCollect().
        andThen(new DriveMeasured(drive, -1).alongWith(shoot.spinWheelsCommand()))/*.alongWith(arm.OpenForIntakeCommand().andThen(intakeLift.OpenCloseIntake(false)))).andThen(arm.goToAngleCommand(0))*/.
        andThen(new WaitCommand(0.3)).
        andThen(transfer.transfer(true).raceWith(roller.shoot())).
        andThen(shoot.stopWheelsCommand().alongWith(transfer.stopWheelsCommand()));        
    }
    public Command shoot()
    {
        return arm.closeSlow().alongWith(shoot.spinWheelsCommand()).
        andThen(new WaitCommand(1.2)).
        andThen(transfer.transfer(true)).
        andThen(shoot.stopWheelsCommand());
    }
    public Command shootMoveOut(){
        return shoot().andThen(moveOut());
    }

    public Command moveOut()
    {
        return new DriveMeasured(drive, 2.5);
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
