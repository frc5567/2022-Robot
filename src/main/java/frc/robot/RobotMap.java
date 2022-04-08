package frc.robot;

public class RobotMap {

    /**
     * timeout in milliseconds for the CTRE config methods
     */
    public static final int TIMEOUT_MS = 30;

    /**
     * Alotted error for turning to a target used only for turning in auton and pilot controller
     */
    public static final double TOLERATED_TARGET_ERROR = 5;

    /**
     * PCM (Pneumatic Control Module) port on the CAN (Controller Area Network) bus
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
    }

    /**
     * Stores constants for Pilot Controller
     */
    public static class PilotControllerConstants{
        /**
         * Port on the competition computer for the pilot controller
         */
        public static final int XBOX_CONTROLLER_PORT = 0;

        /**
         * Constant to record stick deadband that works for our pilot controller (Currently the xbox one controller, but the 360 controllers may need a higher value because of larger drift) 
         */
        public static final double STICK_DEADBAND = 0.09;

        /**
         * Constant for filters on acceleration in order to prevent brownouts
         */
        public static final double SLEW_SIGNAL_RATE_OF_CHANGE = 2.25;

        /**
         * Constant for filters on turning in order to prevent brownouts
         */
        public static final double SLEW_SIGNAL_TURN_RATE_OF_CHANGE = 2.5;

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
         * Port on the competition computer for the copilot controller/gamepad
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
         * Master flywheel falcon ID on the CAN bus
         */
        public static final int MASTER_FLYWHEEL_FALCON_ID = 1;

        /**
         * Slave flywheel falcon ID on the CAN bus
         */
        public static final int SLAVE_FLYWHEEL_FALCON_ID = 2;

        /**
         * Feeder motor ID on the CAN bus
         */
        public static final int FEEDER_MOTOR_ID = 5;

        /**
         * Turret motor ID on the CAN bus
         */
        public static final int TURRET_MOTOR_ID = 3;
        
        /**
         * Constant for feeding speed
         */
        public static final double FEEDING_SPEED = -0.85;

        /**
         * Constant for expel speed
         */
        public static final double EXPEL_SPEED = 0.35;

        /**
         * Constant for turret rotation speed
         */
        public static final double TURRET_ROTATION_SPEED = 0.1;

        /**
         * This is the default target speed we want the flywheel to be moving (in RPM) before launching
         */
        public static final double DEFAULT_TARGET_FLYWHEEL_RPM = 4000;

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
         * Constant for distance from directly on center we allow the target to be.
         */
        public static final double TOLERATED_TURRET_ERROR = 0.8;

        /**
         * Constant for storing the PIDF values for the turret
         */ 
        public static final Gains FLYWHEEL_GAINS = new Gains(0.55, 0.008, 13, 0.045, 100, 1);//Gains(0.57, 0, 16, 0.05, 0, 1);

        /**
         * Constant for the PID slot which allows us to have multiple configurations used in launcher.launchPID
         */
        public static final int PID_SLOT = 2;

        /**
         * Constant for the PID mode
         */
        public static final int PID_MODE = 0;

        /**
         * Constant for the flywheel RPM bound, similar to a deadband for the flywheel RPM.
         */
        public static final int FLYWHEEL_RPM_BOUND = 15;

        /**
         * Constant for the maximum amount of feeding cycles
         */
        public static final double MAX_FEEDING_CYCLES = 8;

        /**
         * Constant for the minimum amount of turret speed
         */
        public static final double MIN_TURRET_SPEED = 0.1;

        /**
         * Constant that records the third angle for turret turning speed
         */
        public static final double TURRET_SPEED_ANGLE_3 = 12;

        /** 
         * Constant that records the second angle for turret turning speed
         */
        public static final double TURRET_SPEED_ANGLE_2 = 8;

        /**
         * Constant that records the first angle for turret turning speed
         */
        public static final double TURRET_SPEED_ANGLE_1 = 3;

        /**
         * Constant that records the second amount of ticks for turret turning speed
         */
        public static final double TURRET_SPEED_TICKS_2 = 3000;

        /** 
         * Constant that records the first amount of ticks for turret turning speed
         */
        public static final double TURRET_SPEED_TICKS_1 = 2000;
        

        /**
         * Contant for the second lowest speed of the turret while targeting
         */
        public static final double LOW_TURRET_SPEED = 0.3;


    }

    /**
     * Stores constants for Intake
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
    }

    /**
     * Constants used in RobotShuffleboard class (also used in Pilot controller when initial scalers are set)
     */
    public static class ShuffleboardConstants {
        /**
         * Default drive input scaler value. Returns if the value is not found from the shuffleboard
         */
        public static final double DRIVE_DEFAULT_INPUT_SCALER = 1.0;

        /**
         * Default flywheel velocity value. Returns if the value is not found from the shuffleboard
         */
        public static final double FLYWHEEL_DEFAULT_PERCENT_POWER = 0.72;

        /**
         * Default auton path. Returns if the value is not found from the shuffleboard
         */
        public static final double DEFAULT_AUTON_PATH = 2;

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
     * Stores constants for Climber
     */
    public static class ClimberConstants{
        /**
         * Constant for the climber motor ID on the CAN bus
         */
        public static final int LIFT_MOTOR_ID = 13;

        /**
         * Constant for the winch motor ID on the CAN bus
         */
        public static final int WINCH_ID = 4;
 
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
         * Constant for the targeting speed in all auton paths
         */
        public static final double TARGETING_SPEED = 0.25;

        /**
         * Number of loops before retracting the intake 
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
         * This value is passed into turnToTarget method in Auton to turn the robot 179 degrees
         */
        public static final double TWO_BALL_STEP_THREE_TARGET_ANGLE = 179;

        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double TWO_BALL_STEP_SIX_TARGET_DISTANCE = 14;

        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 170 degrees
         */
        public static final double TWO_BALL_STEP_SEVEN_TARGET_ANGLE = 170;




        /**
         * The target distances we want out robot to move in step one
         */
        public static final double THREE_BALL_STEP_ONE_TARGET_DISTANCE = 44;

        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 170 degrees
         */
        public static final double THREE_BALL_STEP_THREE_TARGET_ANGLE = 170;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 14 inches
         */
        public static final double THREE_BALL_STEP_SIX_TARGET_DISTANCE = 14;
        
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double THREE_BALL_STEP_SEVEN_TARGET_ANGLE = 60;
        
        /**
         * This value is passed into driveToTarget method in Auton to move the robot 160 inches
         */
        public static final double THREE_BALL_STEP_EIGHT_TARGET_DISTANCE = 59;

        /**
         * This value is passed into driveToTarget method in auton to move the robot 35 inches
         */
        public static final double THREE_BALL_STEP_NINE_TARGET_DISTANCE = 53;
        
        /**
         * This value is passed into turnToTarget method in Auton to turn the robot 110 degrees
         */
        public static final double THREE_BALL_STEP_ELEVEN_TARGET_ANGLE = 75;






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