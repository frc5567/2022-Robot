package frc.robot;

import frc.robot.Auton.AutonPath;

public class RobotMap {

    //timeout in milliseconds for the CTRE config methods
    public static final int TIMEOUT_MS = 30;

    //constant for alotted error for turning to a target used only for turning in auton and pilot controller
    public static final double TOLERATED_TARGET_ERROR = 0.3;

    /**
     * constant for PCM (Pneumatic Control Module) port on the CAN (Controller Area Network) bus
     */
    public static class CANConstants{
        public static final int PCM_CAN_ID = 10;
    }

    /**
    * constants for drivetrain 
    */
    public static class DrivetrainConstants{

        //constants for drivetrain motor IDs (on the CAN - Controller Area Network - bus)
        public static final int MASTER_RIGHT_FALCON_ID = 9; 
        public static final int MASTER_LEFT_FALCON_ID = 6;
        public static final int SLAVE_RIGHT_FALCON_ID = 8;
        public static final int SLAVE_LEFT_FALCON_ID = 7;

        //constants for gearbox solenoid (on the PCM) two ports are needed per double solenoid because the a and b sides are electrically independant
        //Ports are on the PCM 
        public static final int DOUBLESOLENOID_LOW_GEAR_PORT = 6; 
        public static final int DOUBLESOLENOID_HIGH_GEAR_PORT = 7;
        
        //constants for Turn Gains for the PID controller
        public static final Gains TURN_GAINS = new Gains(0.3, 0.0, 0.0, 0.0, 100, 1.0);

        //constants for PID controller
        public static final double PID_INPUT_RANGE = 180.00;
        public static final double ROTATE_PID_INTEGRATOR_RANGE = 0.10;
        public static final double TOLERANCE_ROTATE_CONTROLLER = 2.00;
        //We divide rotational PID output by this number to scale it to match our percent values
        public static final double DRIVE_PID_OUTPUT_SCALAR = 180;

    }

    /**
     * constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        //Port on the comp computer for the pilot controller
        public static final int XBOX_CONTROLLER_PORT = 0;

        //Constant to record stick deadband that works for our pilot controller (Currently the xbox one controller, but the 360 controllers may need a higher value because of larger drift) 
        public static final double STICK_DEADBAND = 0.09;

        //Constants for filters on acceleration and turning in order to prevent brownouts
        public static final double SLEW_SIGNAL_RATE_OF_CHANGE = 0.3;
        public static final double SLEW_SIGNAL_TURN_RATE_OF_CHANGE = 0.4;
    }

    /**
     * constants for Copilot Controller
     */
    public static class CopilotControllerConstants{
        //Port on the comp computer for the copilot controller/gamepad
        public static final int COPILOT_CONTROLLER_PORT = 1;

        //Constant to record stick deadband that works for our copilot controller. This should be tested once we have finalized a copilot controller 
        //TODO Keep this only if an Xbox controller will be used. If a joystick is used on the gamepad, the specific value for that joystick will need to be found
        public static final double STICK_DEADBAND = 0.09;
    }

    /**
     * Constants for Launcher 
     */
    public static class LauncherConstants{
        //constants for drivetrain motor IDs (on the CAN (Continuous Area Network) bus)
        public static final int MASTER_FLYWHEEL_FALCON_ID = 1;
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 2;
        public static final int FEEDER_MOTOR_ID = 5;
        public static final int TURRET_MOTOR_ID = 3;
        
        //These are untested placesholder values until we know what speeds we actually need
        public static final double FEEDING_SPEED = 0.5;
        public static final double EXPEL_SPEED = 0.25;
        public static final double TURRET_ROTATION_SPEED = 0.25;

        //Constant for our turret encoder limit so we don't overturn and damage wiring
        public static final double TURRET_ENCODER_LIMIT = 75000;

        //Constant for distance from directly on center we allow the target to be 
        //TODO Tune when limelight is set up
        public static final double TOLERATED_TURRET_ERROR = 0.08;
        //Constant for how much error we allow the flywheel speed before we launch
        //TODO: This value is a guess and needs to be tuned. Might need to use PID instead of this or adjust the logic
        public static final double TOLERATED_FLYWHEEL_SPEED_ERROR = 0.05;

        //TODO: Change to actual port number once we know what it is
        public static final int LAUNCH_SENSOR_PORT = 10;

        //Constant for the port numbers of the trajectory control pistons two ports are needed per double solenoid because the a and b sides are electrically independant
        //Ports are on the PCM 
        public static final int DOUBLESOLENOID_ANGLE_UP_PORT = 2;
        public static final int DOUBLESOLENOID_ANGLE_DOWN_PORT = 3;
    }

    /**
     * Constants for Intake
     */
    public static class IntakeConstants{
        //constants for intake motor IDs (on the CAN bus)
        public static final int ROLLER_MOTOR_ID = 12;
        public static final int MAGAZINE_MOTOR_ID = 11;
      
        //These are untested placesholder values until we know what speed we actually need
        //TODO These values will need to be tuned
        public static final double ROLLER_SPEED = 0.5;
        public static final double MAGAZINE_SPEED = 1.0;

        //These are untested placesholder values until we know what speed we actually need but it is important that these are negative
        public static final double REVERSE_ROLLER_SPEED = -0.5;
        public static final double REVERSE_MAGAZINE_SPEED = -0.5;

        //constants for solenoids (on the PCM) two ports are needed per double solenoid because the a and b sides are electrically independant 
        //Ports are on the PCM 
        public static final int DOUBLESOLENOID_RETRACTED_PORT = 0;
        public static final int DOUBLESOLENOID_EXTENDED_PORT = 1;

        //TODO: Change to actual port numbers once we know what they are
        //The sensors on the magazine are plugged into ports on the RoboRio
        public static final int MAGAZINE_SENSOR_0_PORT = 9;
        public static final int MAGAZINE_SENSOR_1_PORT = 10;
    }
    
    /**
     * Constants for LimelightVision class
     */
    public static class LimelightConstants{
        //TODO change camera angle after camera is mounted
        public static final double CAMERA_HEIGHT = 23.25; //in inches
        public static final double HUB_HEIGHT = 104.0; //in inches
        public static final double CAMERA_DEGREES_FROM_GROUND = 45;
        // constant for the minimum speed for aiming at target
        //TODO tune the speed we move at
        public static final double MINIMUM_SEEKING_TARGET_SPEED = 0.15;
    }

    /**
     * Constants used in RobotShuffleboard class (also used in Pilot controller when initial scalars are set)
     */
    public static class ShuffleboardConstants {
        public static final double DRIVE_DEFAULT_INPUT_SCALAR = 0.5;
        public static final double FLYWHEEL_DEFAULT_VELOCITY = 0.5;
        public static final double DEFAULT_AUTON_PATH = 0;
    }


    /**
     * Constants for Climber
     */
    public static class ClimberConstants{
        //constants for motor IDs (on the CAN bus)
        public static final int CLIMBER_MOTOR_ID = 13;
        public static final int CLIMBER_WINCH_ID = 4;

        //constants for motor speeds 
        public static final double CLIMBER_MOTOR_SPEED = 0.5;
        public static final double WINCH_MOTOR_SPEED = 0.5;
    }

    /**
     * constants for Auton
     */
    public static class AutonConstants{
        //2048 is the number of ticks per rotation of motor, 21.125 is the circumference of the wheels, 15 and 7.92 are the gear ratios.
        public static final double INCHES_TO_ENCODER_TICKS_LOWGEAR = 2048 / (21.125 / 15);
        public static final double INCHES_TO_ENCODER_TICKS_HIGHGEAR = 2048 / (21.125 / 7.92);

        public static final double DRIVE_SPEED = 0.2;
        public static final double TURN_SPEED = 0.2;
        public static final double TARGETING_SPEED = 0.1;

        //constant for rotation error acceptance
        public static final double ROTATE_BOUND = 0.03;

        public static final double FULL_TURN = 180;
        //This value is passed into driveToTarget method in Auton to move the robot 84.75 inches
        public static final double LEFT_WALL_STEP_ONE_TARGET_DISTANCE = 84.75;
        //TODO: find exact angle, this is a guess
        //This value is passed into turnToTarget method in Auton to turn the robot 30 degrees
        public static final double LEFT_WALL_STEP_TWO_TARGET_ANGLE = 30;
        //This value is passed into driveToTarget method in Auton to move the robot 75 inches 
        public static final double LEFT_WALL_STEP_FOUR_TARGET_DISTANCE = 75;
        //This value is passed into turnToTarget method in Auton to turn the robot 160 degrees 
        public static final double LEFT_WALL_STEP_SIX_TARGET_ANGLE = 160;

        //This value is passed into driveToTarget method in Auton to move the robot 84.75 inches
        public static final double RIGHT_WALL_STEP_ONE_TARGET_DISTANCE = 84.75;
        //TODO: find exact angle, this is a guess
        //This value is passed into turnToTarget method in Auton to turn the robot 30 degrees 
        public static final double RIGHT_WALL_STEP_TWO_TARGET_ANGLE = 30;

        //This value is passed into driveToTarget method in Auton to move the robot 75 inches
        public static final double RIGHT_WALL_STEP_FOUR_TARGET_DISTANCE = 75;
        //This value is passed into turnToTarget method in Auton to turn the robot 160 degrees
        public static final double RIGHT_WALL_STEP_SIX_TARGET_ANGLE = 160;

        //TODO: find exact angle/distance, this is a guess
        //This value is passed into driveToTarget method in Auton to move the robot 48 inches
        public static final double RIGHT_LINE_STEP_TWO_TARGET_DISTANCE = 48;
        //This value is passed into turnToTarget method in Auton to turn the robot 160 degrees
        public static final double RIGHT_LINE_STEP_FOUR_TARGET_ANGLE = 160;
    }
}