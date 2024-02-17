package frc.robot.Subsystems.shooting_subsystems;


import static frc.robot.Constants.ShootingConstants.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.GeneralFunctions;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingSubsystem extends PomSubsystem {


    private final CANSparkMax shooterMotorRight = new CANSparkMax(SHOOTER_MOTOR_RIGHT, CANSparkMax.MotorType.kBrushless);
    private final CANSparkMax shooterMotorLeft = new CANSparkMax(SHOOTER_MOTOR_LEFT, CANSparkMax.MotorType.kBrushless);
    

    private final RelativeEncoder rightEncoder = shooterMotorRight.getEncoder();
    private final RelativeEncoder leftEncoder = shooterMotorLeft.getEncoder();




   public ShootingSubsystem(){
       rightEncoder.setVelocityConversionFactor(1);
       leftEncoder.setPositionConversionFactor(1);
       shooterMotorLeft.follow(shooterMotorRight, true);
       shooterMotorLeft.setIdleMode(IdleMode.kCoast);
       shooterMotorRight.setIdleMode(IdleMode.kCoast);
       setDefaultCommand(this.runOnce(() -> stopMotor()));
       SmartDashboard.putNumber("wanted speed", 0);
   }

   @Override
   public void periodic() {
       SmartDashboard.putNumber("Shooting/Left Motor Speed", leftEncoder.getVelocity());
       SmartDashboard.putNumber("Shooting/Right Motor Speed", rightEncoder.getVelocity());
   }
   

   public boolean atWantedSpeed()
   {
    return GeneralFunctions.allowedError( getRate(), shooterMotorLeft.getAppliedOutput(), TOLERANCE);
   }

    @Override
    public void setMotor(double speed) {
        shooterMotorRight.set(speed);
    }
    @Override
    public void stopMotor() {
        shooterMotorRight.set(0);
    }

    
    public double getRate() {

        return (rightEncoder.getVelocity()+leftEncoder.getVelocity())/2;


    }

    public Command spinWheelsCommand()
    {
        return this.startEnd(() -> setMotor(SHOOT_SPEED), () -> {});
    }
    public Command spinWheelsToSpeedCommand(double speed)
    {
        return this.startEnd(() -> setMotor(speed), () -> {}).until(() -> getRate() >= speed - SHOOT_SPEED_TOLERANCE); 
    }

    public Command stopWheelsCommand()
    {
        return this.runOnce(() -> stopMotor());
    }

    public Command joystickShootCommand(DoubleSupplier sup)
    {
        return run(() -> setMotor(sup.getAsDouble()));
    }
    public Command smartdashShootCommand()
    {
        return run(() -> setMotor(SmartDashboard.getNumber("wanted speed", 0)));
    }

        

}
