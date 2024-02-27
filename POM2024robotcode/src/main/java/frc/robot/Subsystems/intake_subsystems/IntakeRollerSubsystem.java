package frc.robot.Subsystems.intake_subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.PomSubsystem;

import static frc.robot.Constants.IntakeConstants.*;


public class IntakeRollerSubsystem extends PomSubsystem
{
    // Motors
    //------------------------------------------------------------------------------------
    VictorSPX rollerMotor = new VictorSPX(ROLLER_MOTOR);    
    VictorSPX rollerMotorSlave = new VictorSPX(ROLLER_MOTOR_SLAVE);    
    //------------------------------------------------------------------------------------
    
    // the power given to the motor
    double motorSet = 0;
    

    public IntakeRollerSubsystem()
    {
        // setting rollerMotorSlave to follow rollerMotor
        rollerMotorSlave.follow(rollerMotor);
        rollerMotor.setNeutralMode(NeutralMode.Coast);
        rollerMotorSlave.setNeutralMode(NeutralMode.Coast);
        setDefaultCommand(this.runOnce(() -> stopMotor()));
    }
    // the subsystems functions



    @Override
    public void periodic(){
        
    }


    // stoping the motor
    @Override
    public void stopMotor()
    {
        rollerMotor.set(VictorSPXControlMode.PercentOutput,0);
    }

    // setting the motors speed
    @Override
    public void setMotor(double speed)
    {
        rollerMotor.set(VictorSPXControlMode.PercentOutput,speed);
    }


    

    // a boolean function that checks if the color sensor sees a note

    // the subsystems commands
    public Command intakeNoteCommand()
    {
        return new StartEndCommand(() -> setMotor(ROLLER_MOTOR_SPEED), () -> stopMotor(), this);
    }
    public Command outakeNoteCommand()
    {
        return new StartEndCommand(() -> setMotor(-ROLLER_MOTOR_SPEED), () -> stopMotor(), this);
    }

    public Command slow(boolean toShooter)
    {
        return startEnd(() -> setMotor(toShooter ? 0.12 : -0.6), this::stopMotor);
    }
    public Command shoot()
    {
        return startEnd(() -> setMotor(0.2), () -> stopMotor());
    }
}
