package frc.robot.Commands.LedCommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.LedSubsystem;

public class LedCommand extends Command {
    private LedSubsystem m_ledSubsystem;
    private int m_red;
    private int m_green;
    private int m_blue;

    public LedCommand(LedSubsystem ledSubsystem, int red, int green, int blue){ 
        m_ledSubsystem = ledSubsystem;
        m_red = red;
        m_green = green;
    }
    
    @Override
    public void initialize() {
        m_ledSubsystem.setLeds(new Color(m_red, m_green, m_blue));
    }

 
}
