// RobotBuilder Version: 6.1
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public class Constants {
    public static final class JoystickConstants {
        public static final int DRIVE_JOYSTICK = 0;
        public static final int OPERATE_JOYSTICK = 1;

        /** A button axis */
        public static final int A = 1;
        /** B button axis */
        public static final int B = 2;
        /** X button axis */
        public static final int X = 3;
        /** Y button axis */
        public static final int Y = 4;
        /** LB button axis */
        public static final int LB = 5;
        /**RB button axis */
        public static final int RB = 6;
        /**back button axis */
        public static final int BACK = 7;
        /**start button axis */
        public static final int START = 8;

        /** left joystick click button axis */
        public static final int LEFT_JOYSTICK_CLICK = 9;
        /** right joystick click button axis */
        public static final int RIGHT_JOYSTICK_CLICK = 10;

        /** left trigger button axis */
        public static final int LEFT_TRIGGER = 2;
        /** right trigger button axis */
        public static final int RIGHT_TRIGGER = 3;

        public static final int LEFT_JOYSTICK_Y = 1;
        public static final int LEFT_JOYSTICK_X = 0;
        public static final int RIGHT_JOYSTICK_Y = 5;
        public static final int RIGHT_JOYSTICK_X = 4;

        public static final int POV_DOWN = 180;
        public static final int POV_LEFT = 270;
        public static final int POV_RIGHT = 90;
        public static final int POV_UP = 0;
        public static final int POV_NONE = -1;

        public static final double THRESHOLD = 0.3;
    }

    public static final class DriveConstants
    {
        public static final int LEFT_MOTOR_LEAD = 3;
        public static final int LEFT_MOTOR_SLAVE = 4;
        public static final int RIGHT_MOTOR_LEAD = 1;
        public static final int RIGHT_MOTOR_SLAVE = 2;

        public static final int GYRO_ID = 13;

        public static final double TO_RADIANS = 1 / 180 * Math.PI;
        public static final double ROTATIONS_TO_METERS = 1 / 8.45 * 15.24 * Math.PI / 100;

        //all below need to be calculated
        public static final double KS_VOLTS = 0.5;
        public static final double KV_VOLT_SECOND_PER_METER  = 2.8419;
        public static final double KA_VOLT_SECONDS_SQUARE_PER_METER = 1.4226;

        public static final double KP = 0.8;
        public static final double KI = 0.02;
        public static final double KD = 0;
    
        public static final double KP_DRIVE_VEL = 0.002;
        public static final double KTRACK_WIDTH_METERS = 0.555;
        public static final DifferentialDriveKinematics DRIVE_KINEMATICS =
            new DifferentialDriveKinematics(KTRACK_WIDTH_METERS);
    
        public static final double MAX_SPEED_METER_PER_SECOND = 2;
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;

        public static final double RATE = 0.04; 
        public static final double ANGLE_TOLERANCE = 2;

        // const values we dont need to calculate
        public static final double RAMSETE_B = 2;
        public static final double RAMSETE_ZETA = 0.7;

        public static final int TL = 6;
        public static final int BOT_POSE_LEN = 7;
        public static final double SPEAKER_Y = 5.5;
        public static final double FIELD_X = 16;

        public static final int VEL_SLOT = 1;
        public static final double VEL_P = 0.0003;
        public static final double MAX_RPM = 5500;


        
    }

    public static final class ShootingConstants{

        public static final int SHOOTER_MOTOR_RIGHT = 5;
        public static final int SHOOTER_MOTOR_LEFT = 6;
        
        public static final int SHOOTER_ARM_MOTOR = 7;
        public static final int FOLD_MICRO_SWITCH_ID = 3;
        public static final int BRAKE_MICRO_SWITCH_ID = 9;
        
        public static final double KP = 0.9;
        public static final double KI = 0.03;
        public static final double KIZONE = 0;
        public static final double KD = 0.0;

        public static final double SHOOT_KP = 0.00001;
        public static final double SHOOT_KI = 6e-8;

        //trpezoid contraits, need to be found using sysid
        public static final double MAX_VELOCITY_RAD_PER_SECOND = Math.PI;
        public static final double MAX_ACCELERATION_RAD_PER_SECOND_SQUARED = Math.PI;
        public static final double STARTING_OFFSET_RAD = 0;

        //arm feed forward gains, need to be found
        public static final double KS_VOLTS = 0;
        public static final double KG_VOLTS = 0;
        public static final double KV_VOLTS_SECOND_PER_RAD = 0;
        public static final double KA_VOLTS_SECOND_SQUARED_PER_RAD = 0;

        //calculate factors to rads and rads per second
        public static final double POSITON_FACTOR = /*gears*/ 0.25 /*to rads*/ * 2 * Math.PI /*versa*/ /50;
        public static final double VELOCITY_FACTOR = POSITON_FACTOR / 60;


        public static final double NONE = -1;

        public static final double TOLERANCE = 0.017;
        
        public static final double SHOOT_SPEED = 6000;
        public static final double SHOOT_AMP_SPEED = 760;
        public static final double SHOOT_SPEED_TOLERANCE = 0.05;
        public static final double SHOOT_TO_WING_SPEED = 0.8;

        public static final double INTAKE_CAN_MOVE = 0.7;
        public static final double SUB_INTAKE_POS = 0;
        public static final double SHOOT_PODIUM_POS = 0.1;
        public static final double SHOOT_AMP_POS = 1.47;



        public static final double SPEAKER_Y_OFFSET = 5.5;
        // public static final double SPEAKER_X_OFFSET = 0.2915;
        public static final double SPEAKER_HEIGT_WANTED = 2.02;
        public static final double ARM_HEIGT_FROM_FLOOR = 0.4135;
        public static final double ARM_LENGTH = 0.535;
        public static final double SHOOTER_ANGLE_TO_ARM = Math.toRadians(55.34);
        public static final double OTHER_SIDE = 10;
        
        public static final double ARM_OFFSET = 0;

        public static final double CLOSE_SLOW = -0.2;

    }
    

    public static final class TransferConstants
    {
        public static final int TRANSFER_MOTOR = 8;
        public static final double TRANSFER_SHOOT_SPEED =-0.35;
        public static final double TRANSFER_INTAKE_SPEED =-0.1;

        public static final Color noteColor = new Color(130, 98, 26);
        public static final Color blueColor = new Color(54, 113, 86);
        public static final Color[] notNoteColors = new Color[]{new Color(54, 113, 86),
            new Color(142, 110, 28),
            new Color(118, 86, 14),
            new Color(66, 135, 245),
            new Color(15, 18, 15),
            new Color(100,30,70),
            new Color(255,255,255),
            new Color(0,0,0),
            new Color(100,30,255),
            new Color(189, 183, 170)};

        public static final double TRANSFER_TIME_OUT = 1.5;
    }

    public static final class IntakeConstants{
       public static final int ROLLER_MOTOR = 10;
       public static final int ROLLER_MOTOR_SLAVE = 11;

       public static final double INTAKE_SPEED = 0.42;
       public static final double SLOW_OUT = -0.6;
       public static final double SLOW_IN = 0.5;
       public static final double SHOOT = 0.12;

       public static final int LIFT_MOTOR = 12;

        public static final int POTEN_PORTS = 0;
        public static final double POTEN_OFFSET = 0;
        public static final double POTEN_MAX = 5;
        public static final double POTEN_MIN = 4.6;

        public static final double KP = 0.07;
        public static final double KI = 0.007;
        public static final double KD = 0;
        public static final double KG = 0.09;

        public static final double FOLD = 0;
        public static final double GROUND = 3.28;
        public static final double TOLERANCE = 0.15;
        public static final double INTAKE_OFFSET = -6;
        


    }

    public static final class LedsConstants{
        public static final int pRED = 0;
        public static final int pGREEN = 1;
        public static final int pBLUE = 2;
        public static final int NUM_LEDS = 24;
        public static final int LED_PORT = 9;
        public static final Color POM_PURPLE = new Color(98, 0, 151);
        public static final double WAIT_REPEAT = 0.1;
    }

    public static final class ArmLockConstants
    {
        public static final int lockServoChanel = 7;
        public static final double LOCK_ANGLE = 72;
        public static final double OPEN_ANGLE = 100;
    }

    public static final class GeneralFunctions{
        /**
         * checks if you reached set point with an alowed error
         * @param state the current state
         * @param setPoint the set point
         * @param tolerance the allowed error
         * @return have you reached
         */
        public static boolean allowedError(double state, double setPoint, double tolerance)
        {
            return Math.abs(state - setPoint) < tolerance; 
        }
    }

}

