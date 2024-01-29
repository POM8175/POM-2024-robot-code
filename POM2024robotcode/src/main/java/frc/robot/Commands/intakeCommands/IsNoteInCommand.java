package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class IsNoteInCommand extends Command
{
    LedSubsystem ledSubsystem;
    TransferSubsystem transferSubsystem;

    public IsNoteInCommand(LedSubsystem ledSubsystem,TransferSubsystem transferSubsystem)
    {
        this.ledSubsystem = ledSubsystem;
        this.transferSubsystem = transferSubsystem;
        addRequirements(transferSubsystem);
    }

    @Override
    public void execute()
    {
        if(transferSubsystem.isNoteIn()) ledSubsystem.setLeds(Color.kGreen);
        else ledSubsystem.setLeds(Color.kRed);
    }
}
