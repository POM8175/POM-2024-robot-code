package frc.robot.Subsystems.shooting_subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;

import static frc.robot.Constants.ShootingConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.PomSubsystem;

public class ShootingArmSubsystem extends PomSubsystem{
    private final CANSparkMax liftMotor;
    private SparkPIDController pid;
    private DigitalInput foldMicroSwitch;
    private DigitalInput groundMicroSwitch;
    //ShuffleboardTab liftTab = Shuffleboard.getTab("lift");
    
    
  /** Creates a new LiftSubsystem. */
  public ShootingArmSubsystem() {

    liftMotor = new CANSparkMax(SHOOTER_ARM_MOTOR, MotorType.kBrushless);
    pid = liftMotor.getPIDController();
    
    
    pid.setP(KP);
    pid.setI(KI);
    pid.setD(KD);
    pid.setIZone(KIZONE);
    foldMicroSwitch = new DigitalInput(FOLD_MICRO_SWITCH_ID);
    groundMicroSwitch = new DigitalInput(FULL_MICRO_SWITCH_ID);

    SmartDashboard.putNumber("arm encoder", liftMotor.getEncoder().getPosition());
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

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
    return liftMotor.getEncoder().getPosition();
  }

  
  public RelativeEncoder getEncoder(){
    return liftMotor.getEncoder();
  }

  /** set the motors to go to a specified position.
   * @param targetHeight the position to go to
   */
  @Override
  public void setSetPoint(double target) {
    pid.setReference(target, CANSparkMax.ControlType.kPosition);
  }

  

  /**sets the motor to a paramater value
  * @param power the power to set the motor to
  */
  @Override
  public void setMotor(double speed)
  {
    liftMotor.set(speed);
  }

 @Override
  public void stopMotor()
  {
    liftMotor.set(0);
  }

  public CANSparkMax getMotor(){
    return liftMotor;
  }
}
