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
        //TODO switch to the right pipeline
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
        new TurnToDegreeCommand(driveSubsystem, () -> tx.getDouble(0), 0).schedule();;
    }



}
