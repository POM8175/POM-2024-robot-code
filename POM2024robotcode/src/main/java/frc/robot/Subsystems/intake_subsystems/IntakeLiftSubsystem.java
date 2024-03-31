package frc.robot.Subsystems.intake_subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
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
    double goodValue = 0;
    ArmFeedforward feedforward = new ArmFeedforward(0, KG, 0);
    Mechanism2d intakeMec = new Mechanism2d(1, 1);
    MechanismRoot2d root = intakeMec.getRoot("Intake", 2, 0);
    MechanismLigament2d intakeArm = new MechanismLigament2d(getName(), LIFT_MOTOR, INTAKE_SPEED);
    public IntakeLiftSubsystem()
    {
        potentiometer = new AnalogPotentiometer(POTEN_PORTS, 2 * Math.PI, -0.7);
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
        SmartDashboard.putData("Mechanism/Intake",intakeMec);
        SmartDashboard.putNumber("Potentiometer", goodValue);
        SmartDashboard.putNumber("intake arm motor", motor.get());
        SmartDashboard.putBoolean("arm is there", armIsThere.getAsBoolean());
        SmartDashboard.putBoolean("is intake opened", isOpen());
        SmartDashboard.putBoolean("is intake closed", isClosed());
        if(potentiometer.get() > 0 && potentiometer.get() < 4)
        {
            goodValue = potentiometer.get();
        }
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
        return () -> ((goodValue > 0 + TOLERANCE && pid.getSetpoint()  == FOLD) || (goodValue < GROUND - TOLERANCE + TOLERANCE && pid.getSetpoint()  == GROUND));
    }

    public boolean isOpen()
    {
        // return getEncoderPosition() > GROUND / 5 * 4 && pid.getSetpoint() == GROUND;
        return open && motor.get() == 0;
    }
    public boolean isClosed()
    {
        // return getEncoderPosition() < GROUND / 5 && pid.getSetpoint() == FOLD;
        return !open && motor.get() == 0;
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
    SmartDashboard.putNumber("intake pid val", pid.calculate(goodValue));
    motor.set(pid.calculate(goodValue) + feedforward.calculate(goodValue, 0));
  }

    @Override
    public double getEncoderPosition()
    {
        return goodValue + POTEN_OFFSET;
    }

    public Command OpenCloseIntake(boolean open)
    {
        this.open = open;
        return (runOnce(() -> pid.reset()).andThen(run(() -> setSetPoint(open ? GROUND : FOLD))).until(() -> pid.atSetpoint()).andThen(this.runOnce(() -> stopMotor()))).withTimeout(1.8).unless(armIsThere);
    }

    
    public Command OpenCloseIntakeTimers(boolean open)
    {
        return (
            run(() -> {setMotor(open ? 0.24 : -0.26); this.open = open;}).withTimeout(open ? 1 : 1).
            andThen(this.runOnce(() -> setMotor(open ? 0.2: -0.22))).andThen(new WaitCommand(0.1)).
            andThen(this.runOnce(() -> setMotor(open ? 0.05: -0.15))).andThen(new WaitCommand(0.1)).
            andThen(this.runOnce(() -> setMotor(open ? 0.0: 0))).andThen(new WaitCommand(0.1)).
            andThen(this.runOnce(() -> setMotor(open ? -0.1: 0.1))).andThen(new WaitCommand(0.2)).
            andThen(() -> stopMotor())
            ).unless(armIsThere);
    }
    public Command OpenCloseIntakeTimersWithNote(boolean open)
    {
        return (
            run(() -> {setMotor(open ? 0.28 : -0.24); this.open = open;}).withTimeout(open ? 1.65 : 1.5).
            andThen(this.runOnce(() -> setMotor(open ? 0.15: -0.1))).andThen(new WaitCommand(0.1)).
            andThen(this.runOnce(() -> setMotor(open ? -0.1: 0.1))).andThen(new WaitCommand(0.1)).
            andThen(() -> stopMotor())
            ).unless(armIsThere).unless(() -> this.open == open);
    }

    public Command stayInPlace()
    {
        return run(() -> setMotor(feedforward.calculate (goodValue, 0)));
    }
    public Command goToCommand(double to)
    {        
        return runOnce(() -> pid.reset()).andThen(run(() -> setSetPoint(to)).until(() -> pid.atSetpoint()));
    }
}