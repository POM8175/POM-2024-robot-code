package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.LedCommands.LedCommand;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;

public class IsNoteInCommand extends Command
{
    LedSubsystem ledSubsystem;
    IntakeSubsystem intakeSubsystem;
    LedCommand greenLedCommand,redLedCommand;

    public IsNoteInCommand(LedSubsystem ledSubsystem,IntakeSubsystem intakeSubsystem)
    {
        this.ledSubsystem = ledSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
        redLedCommand = new LedCommand(ledSubsystem, 255, 0, 0);
        greenLedCommand = new LedCommand(ledSubsystem, 0, 255, 0);
    }

    
    public void execute()
    {
        if(intakeSubsystem.isNoteIn()) ledSubsystem.setLeds(0, 255, 0);
        else ledSubsystem.setLeds(255, 0, 0);
    }
}
