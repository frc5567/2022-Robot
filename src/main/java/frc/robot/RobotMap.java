package frc.robot;

public class RobotMap {

    //timeout in milliseconds for the CTRE config methods
    public static final int TIMEOUT_MS = 30;

    //constant for alotted error for turning to a target
    public static final double TOLERATED_TARGET_ERROR = 0.3;

    /**
     * constant for PCM location on the CAN bus
     */
    public static class CANConstants{
        public static final int PCM_CAN_ID = 20;
    }

    /**
    * constants for drivetrain 
    */
    public static class DrivetrainConstants{

        //constants for drivetrain motor ID's (on the CAN bus)
        public static final int MASTER_RIGHT_FALCON_ID = 4; 
        public static final int MASTER_LEFT_FALCON_ID = 3;
        public static final int SLAVE_RIGHT_FALCON_ID = 14;
        public static final int SLAVE_LEFT_FALCON_ID = 13;

        //constants for gearbox solenoid (on the PCM)
        public static final int DOUBLESOLENOID_LOW_GEAR_PORT = 4; 
        public static final int DOUBLESOLENOID_HIGH_GEAR_PORT = 5;
        
        //constants for Turn Gains for the PID controller
        public static final Gains TURN_GAINS = new Gains(0.3, 0.0, 0.0, 0.0, 100, 1.0);

        //constants for PID controller
        public static final double PID_INPUT_RANGE = 180.00;
        public static final double ROTATE_PID_INTEGRATOR_RANGE = 0.10;
        public static final double TOLERANCE_ROTATE_CONTROLLER = 2.00;
    }

    /**
     * constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        public static final int XBOX_CONTROLLER_PORT = 0;

        public static final double STICK_DEADBAND = 0.09;

        public static final double SLEW_SIGNAL_RATE_OF_CHANGE = 0.3;
    }

    /**
     * constants for Copilot Controller
     */
    public static class CopilotControllerConstants{
        public static final int GAMEPAD_PORT = 1;

        public static final double STICK_DEADBAND = 0.09;
    }

    /**
     * Constants for Launcher 
     */
    public static class LauncherConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int MASTER_FLYWHEEL_FALCON_ID = 5;
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 6;
        public static final int FEEDER_MOTOR_ID = 7;
        public static final int TURRET_MOTOR_ID = 8;
        public static final int TRAJECTORY_MOTOR_ID = 9;
        
        //These are untested placesholder values until we know what speed we actually need
        public static final double IDLE_SPEED = 0.0;
        public static final double FIRING_SPEED = 1;
        public static final double FEEDING_SPEED = 0.5;
        public static final double TRAJECTORY_MOTOR_SPEED = 0.25;
        public static final double EXPEL_SPEED = 0.25;

        public static final double POSITIVE_TURRET_ROTATION_SPEED = 0.25;
        public static final double NEGATIVE_TURRET_ROTATION_SPEED = -0.25;

        public static final double TRAJECTORY_ENCODER_LIMIT = 30000;

        public static final double TURRET_ENCODER_LIMIT = 75000;

        public static final double TOLERATED_TURRET_ERROR = 0.08;
        public static final double TOLERATED_TRAJECTORY_ERROR = 1000;

        //TODO: Change to actual port number once we know what it is
        public static final int LAUNCH_SENSOR_PORT = 10;
    }

    /**
     * Constants for Intake
     */
    public static class IntakeConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int FRONT_ROLLER_MOTOR_ID = 11;
        public static final int MAGAZINE_MOTOR_ID = 12;
        //These are untested placesholder values until we know what speed we actually need
        public static final double FRONT_ROLLER_SPEED = 0.5;
        public static final double MAGAZINE_SPEED = 0.5;

        //These are untested placesholder values until we know what speed we actually need but it is important that these are negative
        public static final double REVERSE_FRONT_ROLLER_SPEED = -0.5;
        public static final double REVERSE_MAGAZINE_SPEED = -0.5;

        //constants for solenoids (on the PCM)
        public static final int DOUBLESOLENOID_RETRACTED_PORT = 0;
        public static final int DOUBLESOLENOID_EXTENDED_PORT = 1;

        //TODO: Change to actual port number once we know what it is
        public static final int MAGAZINE_SENSOR_PORT = 9;
    }
    
    /**
     * Constants for LimelightVision class
     */
    public static class LimelightConstants{
        //TODO change camera height after camera is mounted
        public static final double CAMERA_HEIGHT = 12.0; //in inches
        public static final double HUB_HEIGHT = 104.0; //in inches
        public static final double CAMERA_DEGREES_FROM_GROUND = 45;
        // constant for the minimum speed for aiming at target
        //TODO tune the speed we move at
        public static final double MINIMUM_SEEKING_TARGET_SPEED = 0.15;
    }

    /**
     * Constants used in RobotShuffelbaard class (also used in Pilot controller when initial scalars are set)
     */
    public static class ShuffleboardConstants {
        public static final double DRIVE_DEFAULT_INPUT_SCALAR = 0.5;
    }


    /**
     * Constants for Climber
     */
    public static class ClimberConstants{
        //constants for motor IDs (on the CAN bus)
        public static final int CLIMBER_MOTOR_ID = 16;
        public static final int CLIMBER_WINCH_ID = 17;

        //constants for motor speeds 
        public static final double CLIMBER_MOTOR_SPEED = 0.8;
        public static final double CLIMBER_MOTOR_REVERSE_SPEED = -0.8;
        public static final double WINCH_MOTOR_SPEED = 0.8;
    }

    /**
     * constants for Auton
     */
    public static class AutonConstants{
        //contant for encoder ticks to inches (pulled from 2021, may be subject to change)
        public static final double INCHES_TO_ENCODER_TICKS_LOWGEAR = 2048 / (21.125 / 15);
        public static final double INCHES_TO_ENCODER_TICKS_HIGHGEAR = 2048 / (21.125 / 7.92);

        public static final double DRIVE_SPEED = 0.2;
        public static final double TURN_SPEED = 0.2;
        public static final double TARGETING_SPEED = 0.1;
        public static final double PLACEHOLDER_VALUE_DISTANCE = 1;
        public static final double PLACEHOLDER_VALUE_ANGLE_CLOCKWISE = 1;
        public static final double PLACEHOLDER_VALUE_ANGLE_COUNTERCLOCKWISE = -1;

        //constant for rotation error acceptance
        public static final double ROTATE_BOUND = 0.03;

        public static final double FULL_TURN = 180;

        public static final double STEP_ONE_TARGET = 60;
        public static final double STEP_TWO_TARGET = 180;
        public static final double STEP_THREE_TARGET = 10;
    }
}