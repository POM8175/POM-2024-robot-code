package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class TurnToSpeaker extends Command {
    public TurnToSpeaker(DriveSubsystem driveSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    DriveSubsystem driveSubsystem;
    @Override
    public void initialize()
    {
        new TurnToDegreeCommand(driveSubsystem, () -> driveSubsystem.calcAngleToSpeaker(), 0).schedule();;
    }




}
