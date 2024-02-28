package frc.robot.Commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.PIComunicator;

public class DriveToNoteCommand extends Command
{
    // the subsystems
    PIComunicator piComunicator;
    DriveSubsystem driveSubsystem;
    
    int[] note = new int[]{};
    int noteYPos = 0,noteXPos = 0;

    public DriveToNoteCommand(PIComunicator piComunicator, DriveSubsystem driveSubsystem)
    {
        this.driveSubsystem = driveSubsystem;
        this.piComunicator = piComunicator;
    }

    @Override
    public void initialize()
    {
        note = piComunicator.getNote();
    }

    @Override
    public void execute()
    {
        note = piComunicator.getNote();

        noteYPos = (note[2] + note[3]) / 2;
        noteXPos = (note[0] + note[1]) / 2;

        driveSubsystem.arcadeDrive(0.5 ,calculateXPower(noteXPos));
    }


    public double calculateXPower(int noteXPos)
    {
        if(Math.abs(noteXPos-160) < 20) 
        {
            return 0;
        }
        return 0.7 * Math.signum(noteXPos - 160);
    }
}
