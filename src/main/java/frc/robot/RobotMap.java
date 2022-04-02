package frc.robot;

public class RobotMap {

    /**
     * timeout in milliseconds for the CTRE config methods
     */
    public static final int TIMEOUT_MS = 30;

    /**
     * Constant for alotted error for turning to a target used only for turning in auton and pilot controller
     */
    public static final double TOLERATED_TARGET_ERROR = 5;

    /**
     * Constant for PCM (Pneumatic Control Module) port on the CAN (Controller Area Network) bus
     */
    public static final int PCM_CAN_ID = 10;

    /**
    * Stores constants for drivetrain 
    */
    public static class DrivetrainConstants{

         /**
         * Master right falcon ID constant on the CAN bus
         */
        public static final int MASTER_RIGHT_FALCON_ID = 9; 

        /**
         * Master left falcon ID constant on the CAN bus
         */
        public static final int MASTER_LEFT_FALCON_ID = 6;

         /**
         * Slave right falcon ID constant on the CAN bus
         */
        public static final int SLAVE_RIGHT_FALCON_ID = 8;

         /**
         * Slace left falcon ID constant on the CAN bus
         */
        public static final int SLAVE_LEFT_FALCON_ID = 7;

        //constants for gearbox solenoid (on the PCM) two ports are needed because the a and b sides are electrically independant
        /**
         * Low gear solenoid port on the PCM 
         */
        public static final int DOUBLESOLENOID_LOW_GEAR_PORT = 6; 

        /**
         * High gear solenoid port on the PCM 
         */
        public static final int DOUBLESOLENOID_HIGH_GEAR_PORT = 7;
        
        /**
         * Constants for Turn Gains for the PID controller
         */
        public static final Gains TURN_GAINS = new Gains(0.3, 0.0, 0.0, 0.0, 100, 1.0);
    }

    /**
     * Stores constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        /**
         * Port on the comp computer for the pilot controller
         */
        public static final int XBOX_CONTROLLER_PORT = 0;

        /**
         * Constant to record stick deadband that works for our pilot controller (Currently the xbox one controller, but the 360 controllers may need a higher value because of larger drift) 
         */
        public static final double STICK_DEADBAND = 0.09;

        /**
         * Constant for filters on acceleration in order to prevent brownouts
         */
        public static final double SLEW_SIGNAL_RATE_OF_CHANGE = 3.0;

        /**
         * Constant for filters on turning in order to prevent brownouts
         */
        public static final double SLEW_SIGNAL_TURN_RATE_OF_CHANGE = 3.0;

        /**
         * The speed that the robot will move forward when the back button on the pilot controller is pressed for climbing
         */
        public static final double CLIMB_CRAWL_SPEED = 0.2;
    }

    /**
     * Stores constants for Copilot Controller
     */
    public static class CopilotControllerConstants{
        /**
         * Port on the comp computer for the copilot controller/gamepad
         */
        public static final int COPILOT_CONTROLLER_PORT = 2;

    }

    /**
     * Stores constants for Launcher 
     */
    public static class LauncherConstants{
        /**
         * Ticks per rotation on the flywheel motors
         */
        public static final int TICKS_PER_ROTATION = 2048;

        /**
         * Constant for the Master flywheel falcon ID on the CAN bus
         */
        public static final int MASTER_FLYWHEEL_FALCON_ID = 1;

        /**
         * Constant for the slave flywheel falcon ID on the CAN bus
         */
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 2;

        /**
         * Constant for the feedern motor ID on the CAN bus
         */
        public static final int FEEDER_MOTOR_ID = 5;

        /**
         * Constant for the turret motor ID on the CAN bus
         */
        public static final int TURRET_MOTOR_ID = 3;
        
        /**
         * Constant for feeding speed
         */
        public static final double FEEDING_SPEED = -0.8;

        /**
         * Constant for expel speed
         */
        public static final double EXPEL_SPEED = 0.35;

        /**
         * Constant for turret rotation speed
         */
        public static final double TURRET_ROTATION_SPEED = 0.1;

        /**
         * Constant for flywheel speed
         */
        public static final double FLYWHEEL_SPEED = 0.7;

        /**
         * This is the target speed we want the flywheel to be moving (in RPM) before launching
         */
        public static final double TARGET_FLYWHEEL_RPM = 4000;

        /**
         * Constant for converting inches to encoder ticks for the turret to be used to determine the limit for how far left or right the turret can rotate.
         * The 2048 is the number of encoder ticks for the motor, 44.019 is the the circumference of the turret in inches, 70 is the gear ratio for the turret. (2048 / (44.019/70)
         */
        public static final double INCHES_TO_ENCODER_TICKS_TURRET = 2048 / (44.019/70);

        /**
         * Constant for our turret encoder limit so we don't overturn and damage wiring. 2.69 (inches) is the maximum distance we want the turret to be able to turn
         */
        //public static final double TURRET_ENCODER_LIMIT = 2.69 * INCHES_TO_ENCODER_TICKS_TURRET;
        public static final double TURRET_ENCODER_LIMIT = 3900;

        /**
         * Constant for our turret encoder band. Similar to a deadband.
         */
        public static final int TURRET_ENCODER_BAND = 50;

        /**
         * Constant for the maximum RPM value that the flywheel can be moving
         */
        public static final double MAX_FLYWHEEL_RPM = 6380;

        /**
         * Constant for distance from directly on center we allow the target to be to the right
         */
        public static final double TOLERATED_TURRET_ERROR_RIGHT = 0.8;

        /**
         * Constant for distance from directly on center we allow the target to be to the left
         */
        public static final double TOLERATED_TURRET_ERROR_LEFT = -0.8;

        /**
         * Constant for how much error we allow the flywheel speed before we launch
         */
        public static final double TOLERATED_FLYWHEEL_SPEED_ERROR = 0.05;

        /**
         * Constant for storing the PIDF values for the turret
         */ 
        public static final Gains FLYWHEEL_GAINS = new Gains(0.55, 0.005, 13, 0.045, 100, 1);//Gains(0.57, 0, 16, 0.05, 0, 1);

        /**
         * Constant for the PID slot which allows us to have multiple configurations used in launcher.launchPID
         */
        public static final int PID_SLOT = 2;

        /**
         * Constant for the PID mode
         */
        public static final int PID_MODE = 0;

        /**
         * Constant for the flywheel RPM bound
         */
        public static final int FLYWHEEL_RPM_BOUND = 15;

        /**
         * Constant representing the default deadband percentage for the turret motor (currently at 4%)
         */
        public static final double TURRET_MOTOR_DEADBAND = 0.04;

        /**
         * Constant for the maximum amount of feeding cycles
         */
        public static final double MAX_FEEDING_CYCLES = 9;

        /**
         * Constant for the maximum amount of turret rotation
         */
        public static final double MAX_TURRET_ROTATION = 22;

        /**
         * Constant for the minimum amount of turret speed
         */
        public static final double MIN_TURRET_SPEED = 0.1;


    }

    /**
     * Constants for Intake
     */
    public static class IntakeConstants{
        /**
         * constants for roller motor ID (on the CAN bus)
         */
        public static final int ROLLER_MOTOR_ID = 12;

        /**
         * constants for magazine motor ID (on the CAN bus)
         */
        public static final int MAGAZINE_MOTOR_ID = 11;
      
        /**
         * Constant for the roller motor speed
         */
        public static final double ROLLER_SPEED = 0.7;

        /**
         * Constant for the magazine motor speed
         */
        public static final double MAGAZINE_SPEED = -0.6;

        /**
         * Constant for the reverse roller motor speed used in unjam
         */
        public static final double REVERSE_ROLLER_SPEED = -0.5;

        //Two ports are needed per double solenoid because the a and b sides are electrically independant 
        /**
         * Constants for the retracted solenoid port on the PCM
         */
        public static final int DOUBLESOLENOID_RETRACTED_PORT = 4;

        /**
         * Constants for the extended solenoid port on the PCM
         */
        public static final int DOUBLESOLENOID_EXTENDED_PORT = 5;

        /**
         * Constant for the magazine sensor 0 port on the RoboRio
         */
        public static final int MAGAZINE_SENSOR_0_PORT = 0;

        /**
         * Constant for the magazine sensor 1 port on the RoboRio
         */
        public static final int MAGAZINE_SENSOR_1_PORT = 1;
    }
    
    /**
     * Stores constants for LimelightVision class
     */
    public static class LimelightConstants{
        /**
         * Constant for the limelight height in inches
         */
        public static final double CAMERA_HEIGHT = 25;

        /**
         * Constant for the hub height in inches
         */
        public static final double HUB_HEIGHT = 107.0;

        /**
         * Constant for the limelight degrees from the ground
         */
        public static final double CAMERA_DEGREES_FROM_GROUND = 57;

        /**
         * Constant for converting an angle into radians (Math.PI) / 180
         */
        public static final double ANGLE_TO_RADIAN_CONVERT = (Math.PI) / 180; 

        /**
         * Constant for the minimum speed for aiming at target
         */
        public static final double MINIMUM_SEEKING_TARGET_SPEED = 0.15;
    }

    /**
     * Constants used in RobotShuffleboard class (also used in Pilot controller when initial scalers are set)
     */
    public static class ShuffleboardConstants {
        /**
         * Default drive input scaler value. Returns if the value is not found from the shuffleboard
         */
        public static final double DRIVE_DEFAULT_INPUT_SCALER = 0.5;

        /**
         * Default flywheel velocity value. Returns if the value is not found from the shuffleboard
         */
        public static final double FLYWHEEL_DEFAULT_PERCENT_POWER = 0.72;

        /**
         * Default auton path. Returns if the value is not found from the shuffleboard
         */
        public static final double DEFAULT_AUTON_PATH = 0;

        /**
         * Default launch preset. Returns if the value is not found from the shuffleboard
         */
        public static final int DEFAULT_LAUNCH_PRESET = 0;

        /**
         * Default maximum turret speed. Returns if the value is not found from the shuffleboard
         */
        public static final double DEFAULT_MAX_TURRET_SPEED = 0.75;

        /**
         * Default proportional constant. Returns if the value is not found from the shuffleboard
         */
        public static final double DEFAULT_PROPORTIONAL_CONSTANT = 0.1;
    }


    /**
     * Constants for Climber
     */
    public static class ClimberConstants{
        /**
         * Constant for the climber motor ID on the CAN bus
         */
        public static final int CLIMBER_MOTOR_ID = 13;

        /**
         * Constant for the winch motor ID on the CAN bus
         */
        public static final int CLIMBER_WINCH_ID = 4;
 
        /**
         * Constant for the climber motor speed
         */
        public static final double CLIMBER_MOTOR_SPEED = 0.7;

        /**
         * Constant for the winch motor speed
         */
        public static final double WINCH_MOTOR_SPEED = 0.7;
    }

    /**
     * Stores constants for Auton
     */
    public static class AutonConstants{
        /**
         * Inches to encoder ticks in low gear equation: 2048 / (21.125 / 15). 
         * 2048 is the number of ticks per rotation of motor, 21.125 is the circumference of the wheels, 15 is the gear ratio.
         */
        public static final double INCHES_TO_ENCODER_TICKS_LOWGEAR = 2048 / (21.125 / 15);

        /**
         * Inches to encoder ticks in high gear equation: 2048 / (21.125 / 7.92. 
         * 2048 is the number of ticks per rotation of motor, 21.125 is the circumference of the wheels, 7.92 is the gear ratio.
         */
        public static final double INCHES_TO_ENCODER_TICKS_HIGHGEAR = 2048 / (21.125 / 7.92);

        /**
         * Constant for the drive speed in all auton paths
         */
        public static final double DRIVE_SPEED = 0.4;

        /**
         * Constant for the turn speed in all auton paths but left wall
         */
        public static final double TURN_SPEED = 0.4;

        /**
         * Constant for the turn speed in the left wall path
         */
        public static final double TURN_SPEED_LEFT_WALL = -0.4;

        /**
         * Constant for the targeting speed in all auton paths
         */
        public static final double TARGETING_SPEED = 0.25;

        /**
         * Number of loops before retracting the intake
         * TODO: remove when if we are not able to launch two game pieces with intake in 
         */
        public static final double INTAKE_WAITING_LOOPS = 30;

        /**
         * Constant for number of loops after a ball is launched to move to the next step
         */
        public static final double LOOPS_AFTER_LAUNCH = 150;

        /**
         * Constant for rotation error acceptance
         */
        public static final double ROTATE_BOUND = 0.02;

        /**
         * Constant for a full 180 degree turn
         */
        public static final double FULL_TURN = 180;

        /**
         * This value is passed into driveToTarget method in Auton to move the robot 45 inches
         */
        public static final double TWO_BALL_STEP_ONE_TARGET_DISTANCE = 45;

        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double TWO_BALL_STEP_SIX_TARGET_DISTANCE = 14;

        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 179 degrees
         */
        public static final double TWO_BALL_STEP_FOUR_TARGET_ANGLE = 179;

        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 179 degrees
         */
        public static final double TWO_BALL_STEP_NINE_TARGET_ANGLE = 179;





        public static final double THREE_BALL_STEP_ONE_TARGET_DISTANCE = 45;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double THREE_BALL_STEP_SIX_TARGET_DISTANCE = 14;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 179 degrees
         */
        public static final double THREE_BALL_STEP_FOUR_TARGET_ANGLE = 170;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double THREE_BALL_STEP_NINE_TARGET_ANGLE = 50;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 160 inches
         */
        public static final double THREE_BALL_STEP_TEN_TARGET_DISTANCE = 55;

        public static final double THREE_BALL_STEP_ELEVEN_TARGET_DISTANCE = 35;

        public static final double THREE_BALL_STEP_TWELVE_TARGET_DISTANCE = 14;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double THREE_BALL_STEP_THIRTEEN_TARGET_ANGLE = 75;






        /**
         * This value is passed into driveToTarget method in Auton to move the robot 45 inches
         */
        public static final double FOUR_BALL_STEP_ONE_TARGET_DISTANCE = 45;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double FOUR_BALL_STEP_SIX_TARGET_DISTANCE = 14;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 179 degrees
         */
        public static final double FOUR_BALL_STEP_FOUR_TARGET_ANGLE = 170;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double FOUR_BALL_STEP_NINE_TARGET_ANGLE = 45;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 160 inches
         */
        public static final double FOUR_BALL_STEP_TEN_TARGET_DISTANCE = 60;

        public static final double FOUR_BALL_STEP_ELEVEN_TARGET_DISTANCE = 35;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double FOUR_BALL_STEP_TWELVE_TARGET_DISTANCE = 14;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double FOUR_BALL_STEP_THIRTEEN_TARGET_ANGLE = 90;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 140 inches
         */
        public static final double FOUR_BALL_STEP_FOURTEEN_TARGET_DISTANCE = 140;

    }

    /**
     * Stores constants for the GamePad
     */
    public static class GamePadConstants {
        /**
         * Constant for the gamepad port on the computer
         */
        public static final int GAMEPAD_PORT = 1;
    }

}