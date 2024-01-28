package frc.robot.Commands.DriveCommands;

import java.util.Optional;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class TurnToSpeaker extends Command {
    public TurnToSpeaker(DriveSubsystem driveSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        table = NetworkTableInstance.getDefault().getTable("limelight-pom");
        tx = table.getEntry("tx");
    }
    DriveSubsystem driveSubsystem;
    Optional<Alliance> ali;
    NetworkTable table;
    NetworkTableEntry tx;
    @Override
    public void initialize()
    {
        TurnToDegreeCommand turn = new TurnToDegreeCommand(driveSubsystem, () -> tx.getDouble(0), 0);
        turn.schedule();
    }



}
