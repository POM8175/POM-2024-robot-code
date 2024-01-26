package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Commands.LedCommands.LedCommand;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;

public class isNoteInCommand extends Command
{
    IntakeSubsystem intakeSubsystem;
    LedSubsystem ledSubsystem;
    LedCommand greenLedCommand = new LedCommand(null, 0, 255, 0);
    LedCommand redLedCommand = new LedCommand(null, 255, 0, 0);
    

    public isNoteInCommand(IntakeSubsystem m_intakeSubsystem,LedSubsystem m_LedSubsystem) 
    {
        this.intakeSubsystem = m_intakeSubsystem;
        this.ledSubsystem = m_LedSubsystem;
    }

    @Override
    public void execute()
    {
        if(intakeSubsystem.isNoteIn()) greenLedCommand.execute();
        else redLedCommand.execute();
    }
}
