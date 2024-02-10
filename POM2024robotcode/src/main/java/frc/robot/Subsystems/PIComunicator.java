package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIComunicator extends SubsystemBase
{
    public PIComunicator()
    {
        
    }

    public boolean canRun()
    {
        return true;
    }

    public int[] getNote()
    {
        return new int[]{};
    }
}