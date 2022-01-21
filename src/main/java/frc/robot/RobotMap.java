package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
    
    /**
    * constants for drivetrain 
    */
    public static class DrivetrainConstants{

        //constants for drivetrain motor ID's (on the CAN bus)
        public static final int MASTER_RIGHT_FALCON_ID = 4; 
        public static final int MASTER_LEFT_FALCON_ID = 3;
        public static final int SLAVE_RIGHT_FALCON_ID = 14;
        public static final int SLAVE_LEFT_FALCON_ID = 13;

        public static final int PCM_CAN_ID = 20;

        //constants for gearbox solenoid (on the PCM)
        public static final int LEFT_DOUBLESOLENOID_LOW_GEAR_PORT = 0;
        public static final int LEFT_DOUBLESOLENOID_HIGH_GEAR_PORT = 1;
        public static final int RIGHT_DOUBLESOLENOID_LOW_GEAR_PORT = 2; 
        public static final int RIGHT_DOUBLESOLENOID_HIGH_GEAR_PORT = 3;

        //timeout in milliseconds for the CTRE config methods
        public static final int TIMEOUT_MS = 30;
    }

    /**
     * constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        public static final int XBOX_CONTROLLER_PORT = 1;

        public static final double PILOT_CONTROLLER_STICK_DEADBAND = 0.08;
    }

    //Constants for Launcher 
    public static class LauncherConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int FLYWHEEL_FALCON_ID = 4;
        public static final int FEEDER_FALCON_ID = 5;
        public static final int TURRET_FALCON_ID = 6;
        
        //These are untested placesholder values until we know what speed we actually need
        public static final double IDLE_SPEED = 0.0;
        public static final double SETUP_SPEED = 0.5;
        public static final double FIRING_SPEED = 1;

        //the launcher timeout for running confing methods
        public static final int CONFIG_TIMEOUT_MS = 30;
    }

    //Constants for Intake
    public static class IntakeConstants{
        //constants for drivetrain motor IDs (on the CAN bus)
        public static final int ROLLER_FALCON_ID = 7;
        public static final int INTAKE_FALCON_ID = 8;
        public static final int MAGAZINE_FALCON_ID = 9;
        //These are untested placesholder values until we know what speed we actually need
        public static final double ROLLER_SPEED = 0.5;
        public static final double INTAKE_SPEED = 0.5;
        public static final double MAGAZINE_SPEED = 0.5;

        //These are untested placesholder values until we know what speed we actually need but it is important that these are negative
        public static final double REVERSE_ROLLER_SPEED = -0.5;
        public static final double REVERSE_INTAKE_SPEED = -0.5;
        public static final double REVERSE_MAGAZINE_SPEED = -0.5;

        public static final int PCM_CAN_ID = 4;

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
    }


}