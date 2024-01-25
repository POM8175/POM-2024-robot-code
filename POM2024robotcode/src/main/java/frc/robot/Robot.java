// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Robot.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.units.Unit;

import static frc.robot.Constants.JoystickConstants.A;

import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import javax.xml.crypto.Data;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.sendable.SendableRegistry;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {
    double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
        /**
     * Converts a raw optical inverse-square reading into a fitted, calibrated linear reading in
     * INCHES.
     */

    DoubleLogEntry Xpose;
    DoubleLogEntry Ypose;
    DoubleLogEntry Rotpose;
    DoubleArrayLogEntry pose;
    double[] posearr = new double[3];

    BooleanLogEntry Abutton;
    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;
    DataLog log = DataLogManager.getLog();
    

    // Color Sensor
    public I2C.Port i2cPort =  I2C.Port.kOnboard;
    public  ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    

    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        m_robotContainer = RobotContainer.getInstance();
        HAL.report(tResourceType.kResourceType_Framework, tInstances.kFramework_RobotBuilder);
        enableLiveWindowInTest(true);

        // colorSensor.configureColorSensor(ColorSensorResolution.kColorSensorRes20bit, ColorSensorMeasurementRate.kColorRate25ms, GainFactor.kGain9x);
                DataLogManager.start();

        // Pose
        Xpose = new DoubleLogEntry(log, "/Pose/X");
        Ypose = new DoubleLogEntry(log, "/Pose/Y");
        Rotpose = new DoubleLogEntry(log, "/Pose/Rot");
        pose = new DoubleArrayLogEntry(log, "Pose");
        Xpose.append(m_robotContainer.driveSubsystem.getPose().getX());
        Ypose.append(m_robotContainer.driveSubsystem.getPose().getY());
        Rotpose.append(m_robotContainer.driveSubsystem.getPose().getRotation().getDegrees());
        posearr[0] = m_robotContainer.driveSubsystem.getPose().getX();
        posearr[1] = m_robotContainer.driveSubsystem.getPose().getY();
        posearr[2] = m_robotContainer.driveSubsystem.getPose().getRotation().getDegrees();

        //Joystick
        Abutton = new BooleanLogEntry(log,"Abtn");
        Abutton.append(m_robotContainer.operateJoystick.getRawButton(A));

        pose.append(posearr);
        

        
//Leds
         m_robotContainer.ledSubsystem.setLeds(colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue());
        

        
  
    }


    ScheduledExecutorService executor = Executors.newScheduledThreadPool(1);
        Runnable task = () -> {
            posearr[0] = m_robotContainer.driveSubsystem.getPose().getX();
            posearr[1] = m_robotContainer.driveSubsystem.getPose().getY();
            posearr[2] = m_robotContainer.driveSubsystem.getPose().getRotation().getDegrees();
            pose.append(posearr);
        };
    /**
    * This function is called every robot packet, no matter the mode. Use this for items like
    * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
    *
    * <p>This runs after the mode specific periodic functions, but before
    * LiveWindow and SmartDashboard integrated updating.
    */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }
    
    
    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        // executor.shutdown();
    }
    
    @Override
    public void disabledPeriodic() {
    }
    
    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }
    
    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }
    
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {  
            m_autonomousCommand.cancel();
        }
        executor.scheduleAtFixedRate(task, 0, 3, TimeUnit.SECONDS);
        m_robotContainer.ledSubsystem.setLeds(colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue());
        
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //Pose Tab
        SmartDashboard.putNumber("Pose/Pose X Value", m_robotContainer.driveSubsystem.getPose().getX());
        SmartDashboard.putNumber("Pose/Pose Y Value", m_robotContainer.driveSubsystem.getPose().getY());
        SmartDashboard.putNumber("Pose/Pose Rotation Value", m_robotContainer.driveSubsystem.getPose().getRotation().getDegrees());


        //Encoder Tab
        SmartDashboard.putNumber("Drive/Encoder/LeftEncoder/Velocity",m_robotContainer.driveSubsystem.getLeftEncoder().getVelocity());
        SmartDashboard.putNumber("Drive/Encoder/RightEncoder/Velocity",m_robotContainer.driveSubsystem.getRightEncoder().getVelocity());
        SmartDashboard.putNumber("Drive/Encoder/LeftEncoder", m_robotContainer.driveSubsystem.getLeftEncoder().getPosition());
        SmartDashboard.putNumber("Drive/Encoder/RightEncoder", m_robotContainer.driveSubsystem.getRightEncoder().getPosition());
        SmartDashboard.putNumber("Drive/Encoder", m_robotContainer.driveSubsystem.getEncoderPosition());


        //Intake Tab
        SmartDashboard.putNumber("Intake/Color/Red", colorSensor.getRed());
        SmartDashboard.putNumber("Intake/Color/Green", colorSensor.getGreen());
        SmartDashboard.putNumber("Intake/Color/Blue", colorSensor.getBlue());
        SmartDashboard.putString("Intake/Color/Color",colorSensor.getColor().toHexString());
        //SmartDashboard.putNumber("Intake/Color/Dist",));
        //SmartDashboard.putNumber("Intake/Color/Dist(no Map)",colorSensor.getProximity());

    // m_robotContainer.ledSubsystem.setLeds(colorSensor.getRed(), colorSensor.getGreen(), colorSensor.getBlue());

    }   

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
    * This function is called periodically during test mode.
    */
    @Override
    public void testPeriodic() {
    }

}