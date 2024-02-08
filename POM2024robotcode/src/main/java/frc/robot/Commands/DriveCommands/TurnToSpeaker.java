package frc.robot.Commands.DriveCommands;

import static frc.robot.Constants.DriveConstants.FIELD_X;
import static frc.robot.Constants.DriveConstants.SPEAKER_Y;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;

public class TurnToSpeaker extends Command {
    public TurnToSpeaker(DriveSubsystem driveSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
        // TODO switch to the right pipeline
        // table = NetworkTableInstance.getDefault().getTable("limelight-pom");
        // tx = table.getEntry("tx");
    }
    DriveSubsystem driveSubsystem;
    Optional<Alliance> ali;
    // NetworkTable table;
    // NetworkTableEntry tx;
    @Override
    public void initialize()
    {
        new TurnToDegreeCommand(driveSubsystem, () -> calcAngle(), 0).schedule();;
    }

    public double calcAngle()
    {
        return Math.tanh((driveSubsystem.getPose().getY() - SPEAKER_Y) / 
        (DriverStation.getAlliance().get() == Alliance.Red ? driveSubsystem.getPose().getX() : FIELD_X - driveSubsystem.getPose().getX()));
    }


}
