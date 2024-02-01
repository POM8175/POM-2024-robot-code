package frc.robot.Subsystems.shooting_subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShootingConstants.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingArmSubsystem extends PomSubsystem{
    private final CANSparkMax liftMotor;
    private final RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private DigitalInput groundMicroSwitch;
    private BooleanSupplier intakeIsThere;
    //ShuffleboardTab liftTab = Shuffleboard.getTab("lift");
      private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          KS_VOLTS, KG_VOLTS,
          KV_VOLTS_SECOND_PER_RAD, KA_VOLTS_SECOND_SQUARED_PER_RAD);

    private final ProfiledPIDController controller;
  /** Creates a new LiftSubsystem. */
  public ShootingArmSubsystem() {
    controller = new ProfiledPIDController (KP, KI, KD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY_RAD_PER_SECOND, MAX_ACCELERATION_RAD_PER_SECOND_SQUARED), 0.02);
    controller.setTolerance(TOLERANCE);

    liftMotor = new CANSparkMax(SHOOTER_ARM_MOTOR, MotorType.kBrushless);
    encoder = liftMotor.getEncoder();
    pid = liftMotor.getPIDController();
    
    encoder.setPositionConversionFactor(POSITON_FACTOR);
    encoder.setVelocityConversionFactor(VELOCITY_FACTOR);
    pid.setP(KP, 0);
    pid.setI(KI, 0);
    pid.setD(KD, 0);
    pid.setIZone(KIZONE, 0);
    foldMicroSwitch = new DigitalInput(FOLD_MICRO_SWITCH_ID);
    groundMicroSwitch = new DigitalInput(FULL_MICRO_SWITCH_ID);

    SmartDashboard.putNumber("arm encoder", encoder.getPosition());
  }

  @Override
  public void periodic() {
    //liftTab.add("arm encoder", liftMotor.getEncoder().getPosition()).withPosition(0, 0).withSize(1, 1).withWidget(BuiltInWidgets.kNumberSlider);
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm encoder", liftMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("arm fold limit switch", isFoldSwitchPressed());
    SmartDashboard.putBoolean("arm ground limit switch", isGroundSwitchPressed());
    if(isFoldSwitchPressed())
    {
      resetEncoder();
    }
    if(isGroundSwitchPressed())
    {
      //liftMotor.getEncoder().setPosition(GROUND);
    }
  }

  public void setIntakeSup(BooleanSupplier sup)
  {
    intakeIsThere = sup;
  }

  /** returns is the fold switch pressed
   * @return the fold limit switch value
   */
  public boolean isFoldSwitchPressed(){
    return !foldMicroSwitch.get();
  }

    /** returns is the ground switch pressed
   * @return the ground limit switch value
   */
  public boolean isGroundSwitchPressed(){
    return !groundMicroSwitch.get();
  }

  /** Resets the encoder to currently read a position of 0. */
  @Override
  public void resetEncoder() {
    liftMotor.getEncoder().setPosition(0);
  }

  /** returns the value of the alternate encoder
   * @return the encoder position
   */
  @Override
  public double getEncoderPosition()
  {
    return encoder.getPosition();
  }

  
  public RelativeEncoder getEncoder(){
    return encoder;
  }

  public void moveWithProfile(TrapezoidProfile.State goal)
  {
    setVoltage(controller.calculate(encoder.getPosition(), goal) 
      + m_feedforward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
  }

  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  @Override
  public void setSetPoint(double target) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition, 0);
  }
  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  public void setSetPoint(double target, double feedforward) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition, 0, feedforward);
  }

  

  /**sets the motor to a paramater value
  * @param power the power to set the motor to
  */
  @Override
  public void setMotor(double speed)
  {
    liftMotor.set(speed);
  }

  public void setVoltage(double volts)
  {
    liftMotor.setVoltage(volts);
  }

  @Override
  public void stopMotor()
  {
    liftMotor.set(0);
  }

  public CANSparkMax getMotor(){
    return liftMotor;
  }

  public Command goToAngleCommand(TrapezoidProfile.State goal)
  {
    return new RunCommand(() -> moveWithProfile(goal), this).until(()-> controller.atGoal()).unless(() -> goal.position < INTAKE_CAN_MOVE && intakeIsThere.getAsBoolean());
  }
  public Command OpenForIntakeCommand()
  {
    return new RunCommand(() -> moveWithProfile(new TrapezoidProfile.State(INTAKE_CAN_MOVE, 0)), this).until(()-> encoder.getPosition() > INTAKE_CAN_MOVE).andThen(goToAngleCommand(new TrapezoidProfile.State(encoder.getPosition(), 0)));
  }
}
