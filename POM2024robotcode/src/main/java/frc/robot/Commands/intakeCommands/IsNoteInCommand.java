package frc.robot.Commands.intakeCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.LedSubsystem;
import frc.robot.Subsystems.intake_subsystems.IntakeRollerSubsystem;

public class IsNoteInCommand extends Command
{
    LedSubsystem ledSubsystem;
    IntakeRollerSubsystem intakeRollerSubsystem;

    public IsNoteInCommand(LedSubsystem ledSubsystem,IntakeRollerSubsystem intakeRollerSubsystem)
    {
        this.ledSubsystem = ledSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        addRequirements(intakeRollerSubsystem);
    }

    @Override
    public void execute()
    {
        if(intakeRollerSubsystem.isNoteIn()) ledSubsystem.setLeds(Color.kGreen);
        else ledSubsystem.setLeds(Color.kRed);
    }
}
