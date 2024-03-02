package frc.robot.Subsystems.intake_subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.PomSubsystem;

public class IntakeLiftSubsystem extends PomSubsystem{
    WPI_VictorSPX motor;
    AnalogPotentiometer potentiometer;
    PIDController pid;
    private BooleanSupplier armIsThere;
    boolean open = false;

    ArmFeedforward feedforward = new ArmFeedforward(0, KG, 0);
    public IntakeLiftSubsystem()
    {
        potentiometer = new AnalogPotentiometer(POTEN_PORTS, 2 * Math.PI * 11.11, -57.73 -6.4);
        motor = new WPI_VictorSPX(LIFT_MOTOR);
        pid = new PIDController(KP, KI, KD);
        pid.setTolerance(TOLERANCE);
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralMode.Coast);
        pid.setSetpoint(FOLD);
        pid.setTolerance(TOLERANCE);
        setDefaultCommand(runOnce(() -> stopMotor()).andThen(new WaitCommand(0.1)));
        // setDefaultCommand(stayInPlace());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Potentiometer", potentiometer.get());
        SmartDashboard.putNumber("intake arm motor", motor.get());
        SmartDashboard.putBoolean("arm is there", armIsThere.getAsBoolean());
        SmartDashboard.putBoolean("is intake opened", isOpen());
        SmartDashboard.putBoolean("is intake closed", isClosed());
    }
    
    public boolean isIntakeOpen()
    {
        return open;
    }

    public boolean atSetpoint()
    {
        return pid.atSetpoint();
    }

    public void setArmSup(BooleanSupplier sup)
    {
      armIsThere = sup;
    }

    public BooleanSupplier armCantMove()
    {
        return () -> ((potentiometer.get() > 0 + TOLERANCE && pid.getSetpoint()  == FOLD) || (potentiometer.get() < GROUND - TOLERANCE + TOLERANCE && pid.getSetpoint()  == GROUND));
    }

    public boolean isOpen()
    {
        return getEncoderPosition() > GROUND / 5 * 4 && pid.getSetpoint() == GROUND;
        // return open && motor.get() == 0;
    }
    public boolean isClosed()
    {
        return getEncoderPosition() < GROUND / 5 && pid.getSetpoint() == FOLD;
        // return !open && motor.get() == 0;
    }
    @Override
    public void stopMotor()
    {
        motor.set(0);
    }

    @Override
    public void setMotor(double speed)
    {
        motor.set(speed);
    }

  @Override
  public void setSetPoint(double target) {
    pid.setSetpoint(target);
    SmartDashboard.putNumber("intake setpoint", pid.getSetpoint());
    SmartDashboard.putNumber("intake pid val", pid.calculate(potentiometer.get()));
    motor.set(pid.calculate(potentiometer.get()) + feedforward.calculate(potentiometer.get(), 0));
  }

    @Override
    public double getEncoderPosition()
    {
        return potentiometer.get() + POTEN_OFFSET;
    }

    public Command OpenCloseIntake(boolean open)
    {
        this.open = open;
        return (runOnce(() -> pid.reset()).andThen(run(() -> setSetPoint(open ? GROUND : FOLD))).until(() -> pid.atSetpoint()).andThen(this.runOnce(() -> stopMotor()))).unless(armIsThere);
    }

    
    public Command OpenCloseIntakeTimers(boolean open)
    {
        this.open = open;
        return (run(() -> setMotor(open ? 0.25 : -0.25)).withTimeout(open ? 1.07 : 1.23).andThen(this.runOnce(() -> setMotor(open ? -0.08: 0.08))).andThen(new WaitCommand(0.25)).andThen(() -> stopMotor())).unless(armIsThere);
    }

    public Command stayInPlace()
    {
        return run(() -> setMotor(feedforward.calculate (potentiometer.get(), 0)));
    }
    public Command goToCommand(double to)
    {        
        return runOnce(() -> pid.reset()).andThen(run(() -> setSetPoint(to)).until(() -> pid.atSetpoint()));
    }
}