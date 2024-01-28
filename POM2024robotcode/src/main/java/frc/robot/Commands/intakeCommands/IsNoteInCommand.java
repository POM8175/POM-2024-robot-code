package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;

public class IsNoteInCommand extends Command
{
    LedSubsystem ledSubsystem;
    IntakeSubsystem intakeSubsystem;

    public IsNoteInCommand(LedSubsystem ledSubsystem,IntakeSubsystem intakeSubsystem)
    {
        this.ledSubsystem = ledSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute()
    {
        if(intakeSubsystem.isNoteIn()) ledSubsystem.setLeds(Color.kGreen);
        else ledSubsystem.setLeds(Color.kRed);
    }
}
