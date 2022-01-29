package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {

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
        public static final int DOUBLESOLENOID_LOW_GEAR_PORT = 0; 
        public static final int DOUBLESOLENOID_HIGH_GEAR_PORT = 1;

        //timeout in milliseconds for the CTRE config methods
        public static final int TIMEOUT_MS = 30;
    }

    /**
     * constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        public static final int XBOX_CONTROLLER_PORT = 0;

        public static final double STICK_DEADBAND = 0.09;
    }

    /**
     * constants for Copilot Controller
     */
    public static class CopilotControllerConstants{
        public static final int GAMEPAD_PORT = 1;

        public static final double STICK_DEADBAND = 0.09;
    }


    //Constants for Launcher 
    public static class LauncherConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int MASTER_FLYWHEEL_FALCON_ID = 5;
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 6;
        public static final int FEEDER_FALCON_ID = 7;
        public static final int TURRET_FALCON_ID = 8;
        
        //These are untested placesholder values until we know what speed we actually need
        public static final double IDLE_SPEED = 0.0;
        public static final double FIRING_SPEED = 1;
        public static final double FEEDING_SPEED = 0.5;
        public static final double TRAJECTORY_MOTOR_SPEED = 0.25;
        public static final double EXPEL_SPEED = 0.25;

        //the launcher timeout for running confing methods
        public static final int CONFIG_TIMEOUT_MS = 30;

        public static final double POSITIVE_TURRET_ROTATION_SPEED = 0.25;
        public static final double NEGATIVE_TURRET_ROTATION_SPEED = -0.25;

        public static final double TRAJECTORY_ENCODER_LIMIT = 30000;
    }

    //Constants for Intake
    public static class IntakeConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int ROLLER_FALCON_ID = 9;
        public static final int INTAKE_FALCON_ID = 10;
        public static final int MAGAZINE_FALCON_ID = 11;
        //These are untested placesholder values until we know what speed we actually need
        public static final double ROLLER_SPEED = 0.5;
        public static final double INTAKE_SPEED = 0.5;
        public static final double MAGAZINE_SPEED = 0.5;

        //These are untested placesholder values until we know what speed we actually need but it is important that these are negative
        public static final double REVERSE_ROLLER_SPEED = -0.5;
        public static final double REVERSE_INTAKE_SPEED = -0.5;
        public static final double REVERSE_MAGAZINE_SPEED = -0.5;

        //constants for gearbox solenoid (on the PCM)
        public static final int LEFT_DOUBLESOLENOID_RETRACTED_PORT = 4;
        public static final int LEFT_DOUBLESOLENOID_EXTENDED_PORT = 5;
        public static final int RIGHT_DOUBLESOLENOID_RETRACTED_PORT = 6; 
        public static final int RIGHT_DOUBLESOLENOID_EXTENDED_PORT = 7;
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
        public static final double MINIMUM_SEEKING_TARGET_SPEED = 0.2;
    }

    /**
     * Constants used in RobotShuffelbaard class (also used in Pilot controller when initial scalars are set)
     */
    public static class ShuffleboardConstants {
        public static final double DRIVE_DEFAULT_INPUT_SCALAR = 0.6;
    }


    //Constants for Climber
    public static class ClimberConstants{
        //constants for motor IDs (on the CAN bus)
        public static final int CLIMBER_MOTOR_ID = 12;
        public static final int CLIMBER_WINCH_ID = 13;

        //constants for motor speeds 
        public static final double CLIMBER_MOTOR_SPEED = 0.8;
        public static final double CLIMBER_MOTOR_REVERSE_SPEED = -0.8;
        public static final double WINCH_MOTOR_SPEED = 0.8;
    }

}