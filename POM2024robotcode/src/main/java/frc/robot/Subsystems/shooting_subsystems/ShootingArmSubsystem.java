package frc.robot.Subsystems.shooting_subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShootingConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class ShootingArmSubsystem extends TrapezoidProfileSubsystem{
    private final CANSparkMax liftMotor;
    private final RelativeEncoder encoder;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private DigitalInput groundMicroSwitch;
    //ShuffleboardTab liftTab = Shuffleboard.getTab("lift");
      private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          KS_VOLTS, KG_VOLTS,
          KV_VOLTS_SECOND_PER_RAD, KA_VOLTS_SECOND_SQUARED_PER_RAD);

    
  /** Creates a new LiftSubsystem. */
  public ShootingArmSubsystem() {
    super(
        new TrapezoidProfile.Constraints(
            MAX_VELOCITY_RAD_PER_SECOND, MAX_ACCELERATION_RAD_PER_SECOND_SQUARED),
        STARTING_OFFSET_RAD
        );

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
    super.periodic();
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

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    setSetPoint(setpoint.position, feedforward / 12.0);
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
  public void resetEncoder() {
    liftMotor.getEncoder().setPosition(0);
  }

  /** returns the value of the alternate encoder
   * @return the encoder position
   */
  public double getEncoderPosition()
  {
    return encoder.getPosition();
  }

  
  public RelativeEncoder getEncoder(){
    return encoder;
  }

  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  public void setSetPoint(double target) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition);
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
  public void setMotor(double speed)
  {
    liftMotor.set(speed);
  }

  public void stopMotor()
  {
    liftMotor.set(0);
  }

  public CANSparkMax getMotor(){
    return liftMotor;
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return this.runOnce(() -> setGoal(kArmOffsetRads));
  }
}
