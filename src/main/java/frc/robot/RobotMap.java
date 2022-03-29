package frc.robot;

public class RobotMap {

    //timeout in milliseconds for the CTRE config methods
    public static final int TIMEOUT_MS = 30;

    //constant for alotted error for turning to a target used only for turning in auton and pilot controller
    public static final double TOLERATED_TARGET_ERROR = 5;

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
        public static final double DRIVE_PID_OUTPUT_SCALER = 180;

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
        public static final double SLEW_SIGNAL_RATE_OF_CHANGE = 3.0;
        public static final double SLEW_SIGNAL_TURN_RATE_OF_CHANGE = 3.0;
    }

    /**
     * constants for Copilot Controller
     */
    public static class CopilotControllerConstants{
        //Port on the comp computer for the copilot controller/gamepad
        public static final int COPILOT_CONTROLLER_PORT = 2;

        //Constant to record stick deadband that works for our copilot controller. This should be tested once we have finalized a copilot controller 
        //TODO Keep this only if an Xbox controller will be used. If a joystick is used on the gamepad, the specific value for that joystick will need to be found
        public static final double STICK_DEADBAND = 0.09;


    }

    /**
     * Constants for Launcher 
     */
    public static class LauncherConstants{
        // Ticks per revolution on the flywheel motors
        public static final int TICKS_PER_ROTATION = 2048;
        //constants for drivetrain motor IDs (on the CAN (Continuous Area Network) bus)
        public static final int MASTER_FLYWHEEL_FALCON_ID = 1;
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 2;
        public static final int FEEDER_MOTOR_ID = 5;
        public static final int TURRET_MOTOR_ID = 3;
        
        //These are untested placesholder values until we know what speeds we actually need
        public static final double FEEDING_SPEED = -0.8;
        public static final double EXPEL_SPEED = 0.35;
        public static final double TURRET_ROTATION_SPEED = 0.1;
        public static final double FLYWHEEL_SPEED = 0.7;

        // This is the target speed we want the flywheel to be moving (in RPM) before launching
        // This value should be tested. Currently the 5104 is 80% of the power
        public static final double TARGET_FLYWHEEL_SPEED = 1900;

        //Constant for converting inches to encoder ticks for the turret to be used to determine the limit for how far left or right the turret can rotate
        // the 4096 is the number of encoder ticks for the motor, 44.019 is the the circumference of the turret in inches, 70 is the gear ratio for the turret
        public static final double INCHES_TO_ENCODER_TICKS_TURRET = 2048 / (44.019/70);

        //Constant for our turret encoder limit so we don't overturn and damage wiring.
        //2.69 (inches) is the maximum distance we want the turret to be able to turn
        //public static final double TURRET_ENCODER_LIMIT = 2.69 * INCHES_TO_ENCODER_TICKS_TURRET;
        public static final double TURRET_ENCODER_LIMIT = 5000;

        public static final int TURRET_ENCODER_BAND = 25;

        // This is the max RPM value that the flywheel can be moving
        public static final double MAX_FLYWHEEL_RPM = 6380;

        //Constant for distance from directly on center we allow the target to be 
        //TODO Tune (This was 0.4 and 2.0, change back later
        public static final double TOLERATED_TURRET_ERROR_RIGHT = 16;
        public static final double TOLERATED_TURRET_ERROR_LEFT = 0;

        //Constant for how much error we allow the flywheel speed before we launch
        //TODO: This value is a guess and needs to be tuned. Might need to use PID instead of this or adjust the logic
        public static final double TOLERATED_FLYWHEEL_SPEED_ERROR = 0.05;

        //TODO: Change to actual port number once we know what it is
        public static final int LAUNCH_SENSOR_PORT = 10;

        //Constant for the port numbers of the trajectory control pistons two ports are needed per double solenoid because the a and b sides are electrically independant
        //Ports are on the PCM 
        public static final int DOUBLESOLENOID_ANGLE_UP_PORT = 2;
        public static final int DOUBLESOLENOID_ANGLE_DOWN_PORT = 3;

        // Constant for storing the PID values for the turret
        public static final Gains FLYWHEEL_GAINS = new Gains(0, 0, 0, 0.05, 0, 1);
        //sets the feedback sensor to be using a primary closed loop (0 = primary closed-loop, 1 = auxilary closed-loop)
        public static final int PID_LOOP_IDX = 0;

        public static final double NEUTRAL_DEADBAND = 0.001;

        public static final int PID_SLOT = 2;
        public static final int PID_MODE = 0;

        public static final int FLYWHEEL_RPM_BOUND = 50;
        //Constant representing the default deadband percentage for the turret motor (currently at 4%)
        public static final double TURRET_MOTOR_DEADBAND = 0.04;

        public static final double MAX_FEEDING_CYCLES = 15;

        public static final double MAX_TURRET_ROTATION = 22;

        public static final double MIN_TURRET_SPEED = 0.1;


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
        public static final double ROLLER_SPEED = 0.7;
        public static final double MAGAZINE_SPEED = -0.6;

        //These are untested placesholder values until we know what speed we actually need but it is important that these are negative
        public static final double REVERSE_ROLLER_SPEED = -0.5;
        public static final double REVERSE_MAGAZINE_SPEED = -0.5;

        //constants for solenoids (on the PCM) two ports are needed per double solenoid because the a and b sides are electrically independant 
        //Ports are on the PCM 
        public static final int DOUBLESOLENOID_RETRACTED_PORT = 4;
        public static final int DOUBLESOLENOID_EXTENDED_PORT = 5;

        //The sensors on the magazine are plugged into ports on the RoboRio
        public static final int MAGAZINE_SENSOR_0_PORT = 1;
        public static final int MAGAZINE_SENSOR_1_PORT = 0;
    }
    
    /**
     * Constants for LimelightVision class
     */
    public static class LimelightConstants{
        //TODO change camera angle after camera is mounted
        public static final double CAMERA_HEIGHT = 25; //in inches
        public static final double HUB_HEIGHT = 107.0; //in inches
        public static final double CAMERA_DEGREES_FROM_GROUND = 57;

        //Constant for converting an angle into radians
        public static final double ANGLE_TO_RADIAN_CONVERT = (Math.PI) / 180; 
        // constant for the minimum speed for aiming at target
        //TODO tune the speed we move at
        public static final double MINIMUM_SEEKING_TARGET_SPEED = 0.15;
    }

    /**
     * Constants used in RobotShuffleboard class (also used in Pilot controller when initial scalers are set)
     */
    public static class ShuffleboardConstants {
        public static final double DRIVE_DEFAULT_INPUT_SCALER = 0.5;
        public static final double FLYWHEEL_DEFAULT_VELOCITY = 0.5;
        public static final double DEFAULT_AUTON_PATH = 0;
        public static final int DEFAULT_LAUNCH_PRESET = 0;
        public static final double DEFAULT_MAX_TURRET_SPEED = 0.75;
        public static final double DEFAULT_PROPORTIONAL_CONSTANT = 0.1;
    }


    /**
     * Constants for Climber
     */
    public static class ClimberConstants{
        //constants for motor IDs (on the CAN bus)
        public static final int CLIMBER_MOTOR_ID = 13;
        public static final int CLIMBER_WINCH_ID = 4;

        //constants for motor speeds 
        public static final double CLIMBER_MOTOR_SPEED = 0.7;
        public static final double WINCH_MOTOR_SPEED = 0.7;
    }

    /**
     * constants for Auton
     */
    public static class AutonConstants{
        //2048 is the number of ticks per rotation of motor, 21.125 is the circumference of the wheels, 15 and 7.92 are the gear ratios.
        public static final double INCHES_TO_ENCODER_TICKS_LOWGEAR = 2048 / (21.125 / 15);
        public static final double INCHES_TO_ENCODER_TICKS_HIGHGEAR = 2048 / (21.125 / 7.92);

        public static final double DRIVE_SPEED = 0.4;
        public static final double INTAKE_DRIVE_SPEED = 0.1;
        public static final double TURN_SPEED = 0.4;
        public static final double TURN_SPEED_LEFT_WALL = -0.4;
        public static final double TARGETING_SPEED = 0.25;

        public static final double INTAKE_WAITING_LOOPS = 30;
        //constant for number of loops after a ball is launched to move to the next step
        public static final double LOOPS_AFTER_LAUNCH = 120;

        //constant for rotation error acceptance
        public static final double ROTATE_BOUND = 0.02;

        public static final double FULL_TURN = 180;

        //This value is passed into driveToTarget method in Auton to move the robot 45 inches
        public static final double TWO_BALL_STEP_ONE_TARGET_DISTANCE = 45;
        //This value is passed into driveToTarget method in Auton to move the robot 14 inches
        public static final double TWO_BALL_STEP_SIX_TARGET_DISTANCE = 14;
        //This value is passed into turnToTarget method in Auton to turn the robot 180 degrees
        public static final double TWO_BALL_STEP_FOUR_TARGET_ANGLE = 179;

        public static final double TWO_BALL_STEP_NINE_TARGET_ANGLE = 179;

            //This value is passed into driveToTarget method in Auton to move the robot 45 inches
            public static final double FOUR_BALL_STEP_ONE_TARGET_DISTANCE = 45;
            //This value is passed into driveToTarget method in Auton to move the robot 14 inches
            public static final double FOUR_BALL_STEP_SIX_TARGET_DISTANCE = 14;
            //This value is passed into turnToTarget method in Auton to turn the robot 180 degrees
            public static final double FOUR_BALL_STEP_FOUR_TARGET_ANGLE = 179;
            //This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
            public static final double FOUR_BALL_STEP_NINE_TARGET_ANGLE = 110;
            //This value is passed into driveToTarget method in Auton to move the robot MANY inches
            public static final double FOUR_BALL_STEP_TEN_TARGET_DISTANCE = 160;
            //This value is passed into driveToTarget method in Auton to move the robot many less inches
            public static final double FOUR_BALL_STEP_TWELVE_TARGET_DISTANCE = 14;
            //This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
            public static final double FOUR_BALL_STEP_THIRTEEN_TARGET_ANGLE = 110;
            //This value is passed into driveToTarget method in Auton to move the robot slightly less than step ten inches
            public static final double FOUR_BALL_STEP_FOURTEEN_TARGET_DISTANCE = 140;

        public static final double AUTON_FLYWHEEL_POWER = 0.72;
    }

    /**
     * Stores constants for the GamePad
     */
    public static class GamePadConstants {
        // This is the port for the GamePad
        // TODO chang this port to be 1 after the xbox controller for the Copilot controller is no longer being used
        public static final int GAMEPAD_PORT = 1;
    }

}