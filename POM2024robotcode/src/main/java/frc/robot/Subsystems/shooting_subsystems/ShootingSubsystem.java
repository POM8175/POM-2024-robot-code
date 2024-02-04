package frc.robot.Subsystems.shooting_subsystems;


import static frc.robot.Constants.DriveConstants.ROTATIONS_TO_METERS;
import static frc.robot.Constants.ShootingConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingSubsystem extends PomSubsystem {


    private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
    

    private final RelativeEncoder rightEncoder = shooterMotorRight.getEncoder();
    private final RelativeEncoder leftEncoder = shooterMotorLeft.getEncoder();




   public ShootingSubsystem(){
       rightEncoder.setVelocityConversionFactor(ROTATIONS_TO_METERS);
       leftEncoder.setPositionConversionFactor(ROTATIONS_TO_METERS);
       shooterMotorLeft.follow(shooterMotorRight, true);
       shooterMotorLeft.setIdleMode(IdleMode.kCoast);
       shooterMotorRight.setIdleMode(IdleMode.kCoast);
       setDefaultCommand(this.runOnce(() -> stopMotor()));
   }
   public void periodic(){

    Shuffleboard.getTab("Shooter").addNumber("Shooter Velocity", () -> getRate());

   }

   public boolean atWantedSpeed()
   {
    return getRate() >= shooterMotorLeft.getAppliedOutput();
   }

    @Override
    public void setMotor(double speed) {
        shooterMotorRight.set(speed);
    }

    public double getRate() {

        return (rightEncoder.getVelocity()+leftEncoder.getVelocity())/2;


    }

    public Command spinWheelsCommand()
    {
        return new StartEndCommand(() -> setMotor(SHOOT_SPEED), null, this).until(() -> getRate() >= SHOOT_SPEED);
    }
    public Command spinWheelsToSpeedCommand(double speed)
    {
        return new StartEndCommand(() -> setMotor(speed), null, this).until(() -> getRate() >= speed); 
    }

    public Command stopWheelsCommand()
    {
        return this.runOnce(() -> stopMotor());
    }

        

}
