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
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Subsystems.PomSubsystem;

public class IntakeLiftSubsystem extends PomSubsystem{
    WPI_VictorSPX motor;
    AnalogPotentiometer potentiometer;
    PIDController pid;
    private BooleanSupplier armIsThere;
    boolean open = false;

    ArmFeedforward feedforward = new ArmFeedforward(0, 0.1, 0);
    public IntakeLiftSubsystem()
    {
        potentiometer = new AnalogPotentiometer(POTEN_PORTS, 2 * Math.PI, -1.335);
        motor = new WPI_VictorSPX(LIFT_MOTOR);
        pid = new PIDController(KP, KI, KD);
        pid.setTolerance(TOLERANCE);
        motor.clearStickyFaults();
        motor.setNeutralMode(NeutralMode.Brake);
        pid.setSetpoint(FOLD);
        setDefaultCommand(stayInPlace());
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Potentiometer", potentiometer.get());
        SmartDashboard.putNumber("intake arm motor", motor.get());
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

    public BooleanSupplier armCanMove()
    {
        return () -> !((potentiometer.get() > 0 + TOLERANCE && pid.getSetpoint()  == FOLD) || (potentiometer.get() < GROUND - TOLERANCE + TOLERANCE && pid.getSetpoint()  == GROUND));
    }

    public boolean isOpen()
    {
        return getEncoderPosition() > GROUND / 5 * 4 && pid.getSetpoint() == GROUND;
    }
    public boolean isClosed()
    {
        return getEncoderPosition() < GROUND / 5 && pid.getSetpoint() == FOLD;
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
    motor.set(pid.calculate(potentiometer.get()) + feedforward.calculate(target, 0));
  }

    @Override
    public double getEncoderPosition()
    {
        return potentiometer.get() + POTEN_OFFSET;
    }

    public Command OpenCloseIntake(boolean open)
    {
        this.open = open;
        pid.setSetpoint(open ? GROUND : FOLD);
        return( new RunCommand(() -> setMotor(pid.calculate(getEncoderPosition())), this).until(() -> pid.atSetpoint()).andThen(this.runOnce(() -> stopMotor()))).unless(armIsThere);
    }

    public Command stayInPlace()
    {
        return run(() -> setMotor(feedforward.calculate (potentiometer.get(), 0)));
    }
    public Command goToCommand(double to)
    {
        return run(() -> setSetPoint(to));
    }
}
