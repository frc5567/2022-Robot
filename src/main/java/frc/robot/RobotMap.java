package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class RobotMap {
    
    /**
    * constants for drivetrain 
    */
    public static class DrivetrainConstants{

        //constants for drivetrain motor ID's (on the CAN bus)
        public static final int MASTER_RIGHT_FALCON_ID = 0; 
        public static final int MASTER_LEFT_FALCON_ID = 1;
        public static final int SLAVE_RIGHT_FALCON_ID = 2;
        public static final int SLAVE_LEFT_FALCON_ID = 3;

        public static final int PCM_CAN_ID = 4;

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
        //These are untested placesholder values until we know what speed we actually need
        public static final double SETUP_SPEED = 0.5;
        public static final double FIRING_SPEED = 1;

        //the launcher timeout for running confing methods
        public static final int CONFIG_TIMEOUT_MS = 30;
    }

}