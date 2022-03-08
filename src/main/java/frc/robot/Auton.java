package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Intake.IntakeState;

public class Auton{
    //enum for storing what path we are going to take in auton
    public enum AutonPath{
        //auton path for starting on the hub wall on the left side from the drivers' station perspective
        kLeftWall,

        //auton path for starting on the hub wall on the right side from the drivers' station perspective
        kRightWall,

        //auton path for starting on the edge of the tarmac line on the right side from the drivers' station perspective
        kRightLine;
    }

    //enum for each of the steps in our auton
    public enum AutonStep{
        kStep1,

        kStep2,

        kStep3,

        kStep4,

        kStep5,

        kStep6,

        kStep7,

        kStep8,

        kStep9,

        kStop;
    }
    
    //this is a quick and dirty workaround because the sensors are not functional at the moment
    private int m_loopCount = 0; 

    //declares variables for the auton class
    //Enums to store what step we are on and what path we chose
    private AutonStep m_step;
    private AutonPath m_path;

    //Member variables to store the systems we pass in
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelightVision;

    //declares shuffleboard to be used for path selection
    private RobotShuffleboard m_shuffleboard;
    private double m_currentAutonPath = RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH;

    //Varaible that is only true if we have a second ball to launch
    boolean m_secondBall = false;
    //Variable that is only true the first time through the auton periodic loop to print out that we have started auton
    boolean m_autonStartFlag = true;
    //Variable to store the current state of our Limelight LEDS
    boolean m_limelightOff = true;
    //Variable to store whether or not we can currently see a target
    boolean m_canSeeTarget = false;
    //Boolean for Sysout counter
    boolean m_doSysOut = true;

    //Variables for Sysouts
    double m_targetEncoderTicks;
    double m_currentRightEncoderTicks;
    double m_currentLeftEncoderTicks;

    //Stores the value from the xAngleToTarget method
    double m_xToTarget;

    int m_sysOutCounter;

    /**
     * constructor for auton
     * @param drivetrain we pass in drivetrain to be able to drive around in auton
     * @param launcher we pass in the launcher to be able to launch during auton
     * @param intake we pass in the intake to be able to pick up game pieces during auton
     * @param limelightVision we pass in limelight to be able to target during auton
     */
    public Auton(Drivetrain drivetrain, Launcher launcher, Intake intake, LimelightVision limelightVision, RobotShuffleboard shuffleboard){
        m_shuffleboard = shuffleboard;
        m_drivetrain = drivetrain;
        m_launcher = launcher;
        m_intake = intake;
        m_limelightVision = limelightVision;

        //Sets the current step to step 1 and the path to whatever path we are currently running
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kRightWall;
    }

    /**
     * this method will be run at the start of every auton period
     */
    public void init(){
        m_drivetrain.init();
        m_launcher.init();
        m_intake.init();
        m_limelightVision.init();
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kRightWall;
        m_drivetrain.zeroEncoders();
        m_currentAutonPath = m_shuffleboard.getAutonPath();
    }

    /**
     * this method will be called many times a second during the auton period
     */
    public void periodic(){
        //Runs Shuffleboard periodic to update our shuffleboard controlled values in real time
        m_shuffleboard.periodic();
        //Sets our current path to the one entered on the shuffleboard
        selectPath();
        //calls the limelight periodic method in order to update the network tables every cycle
        m_limelightVision.periodic();
        //calls the intake periodic method for automatic indexing
        //Commented out for safety and testing purposes
        //m_intake.periodic();

        m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();

        int CurrentLEDStatus = m_limelightVision.currentLEDStatus();
        //If the Limelight is off, sets m_limelightOff to be true
        if(CurrentLEDStatus == 1){
            m_limelightOff = true;
        }
        //If the Limelight is on, sets m_limelightOff to be false
        else if(CurrentLEDStatus == 3){
            m_limelightOff = false;
        }
        else{
            System.out.println("unknown LED status");
        }

        //Prints out "Starting Auton" only once
        if(m_autonStartFlag){
            System.out.println("STARTING AUTON");
            m_autonStartFlag = false;
        }
 
        //Counter for sysouts
        if((m_sysOutCounter % 50) == 0){
            m_doSysOut = true;
        }
        else{
            m_doSysOut = false;
        }

        if(m_doSysOut == true){
            System.out.println("Right Encoder Ticks: " + m_currentRightEncoderTicks);
            System.out.println("Left Encoder Ticks: " + m_currentLeftEncoderTicks);   
        }

        //Starts auton pathing in one of our three paths: Left Wall, Right Wall, or Right Line
        if (m_path == AutonPath.kLeftWall){
            if(m_step == AutonStep.kStep1){
                //Activates our drive to target method, which returns true once we have reached our target
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_ONE_TARGET_DISTANCE)){
                    //once drive to target returns true, zero encoders and drivetrain to stop the bot and prepare the sensors for the next step
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                //Activates our turn to angle method, which returns true once we have reached our target
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_TWO_TARGET_ANGLE)){
                    //when turn to angle returns true, zero encoders, gyro and drivetrain to stop the robot and prepare the sensors for the next step
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep3){
                //extends intake to prepare for the next step, where we activate our intake
                m_intake.setIntakeExtension(IntakeState.kExtended);
                System.out.println("Intake Extended");
                m_step = AutonStep.kStep4;
            }
            if(m_step == AutonStep.kStep4){
                //Activates intake and drives forward to a target in order to pick up a game piece
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_FOUR_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep5){
                //Sysout to tell drivers if we acually picked up a game piece
                System.out.println("Game Piece Picked up:" + m_intake.getMagazineSensor1());
                //Zeros roller speed before we retract the intake
                m_intake.setRollerSpeed(0);
                //Retract intake so that we can continue our path without risking breaking the intake system
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                System.out.println("Intake Retracted");
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                //Turns the robot a full 180 degrees in order to face the hub
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_SIX_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep7;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    m_limelightVision.enableLEDs();
                }
                //This is all currently placeholder code for vision/targeting testing, it will eventually be replaced with m_launcher.launch()
                //Stores the current distance from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                //If we can see the target, move on to the next logic
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen, send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.periodic(0,0);
                        m_drivetrain.zeroEncoders();
                        m_limelightVision.disableLEDs();
                        m_step = AutonStep.kStop;
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.periodic(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightWall){
            if(m_step == AutonStep.kStep1){
                //Activates our drive to target method, which returns true once we have reached our target
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_ONE_TARGET_DISTANCE)){
                    //once drive to target returns true, zero encoders and drivetrain to stop the bot and prepare the sensors for the next step
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                //Activates our turn to angle method, which returns true once we have reached our target
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_TWO_TARGET_ANGLE)){
                    //when turn to angle returns true, zero encoders, gyro and drivetrain to stop the robot and prepare the sensors for the next step
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep3){
                //extends intake to prepare for the next step, where we activate our intake
                m_intake.setIntakeExtension(IntakeState.kExtended);
                System.out.println("Intake Extended");
                m_step = AutonStep.kStep4;
            }
            if(m_step == AutonStep.kStep4){
                //Activates intake and drives forward to a target in order to pick up a game piece
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_FOUR_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep5){
                //Retract intake so that we can continue our path without risking breaking the intake system
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                System.out.println("Intake Retracted");
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                //Turns the robot a full 180 degrees in order to face the hub
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_SIX_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep7;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    m_limelightVision.enableLEDs();
                }
                //This is all currently placeholder code for vision/targeting testing, it will eventually be replaced with m_launcher.launch()
                //Stores the current distance from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                //If we can see the target, move on to the next logic
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen, send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.periodic(0,0);
                         m_drivetrain.zeroEncoders();
                         m_limelightVision.disableLEDs();
                         m_step = AutonStep.kStop;
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                            System.out.println("No target detected");    
                        }
                }
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.periodic(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightLine){ 
            if(m_step == AutonStep.kStep1){
                //extends intake to prepare for the next step, where we activate our intake
                m_intake.setIntakeExtension(IntakeState.kExtended);
                System.out.println("Intake Extended");
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                //Activates intake and drives forward to a target in order to pick up a game piece
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_LINE_STEP_TWO_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep3){
                //Retract intake so that we can continue our path without risking breaking the intake system
                //m_intake.setIntakeExtension(IntakeState.kRetracted);
                System.out.println("Intake Retracted");
                m_step = AutonStep.kStep4;
            }
            else if(m_step == AutonStep.kStep4){
                //Turns the robot a full 180 degrees in order to face the hub
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_LINE_STEP_FOUR_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                    m_launcher.setFlywheelSpeed(RobotMap.LauncherConstants.FLYWHEEL_SPEED);
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep5){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    m_limelightVision.enableLEDs();
                }
                //This is all currently placeholder code for vision/targeting testing, it will eventually be replaced with m_launcher.launch()
                //Stores the current distance from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                //If we can see the target, move on to the next logic
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen, send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.periodic(0,0);
                        m_drivetrain.zeroEncoders();
                        m_limelightVision.disableLEDs();
                        m_step = AutonStep.kStop;
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            else if (m_step == AutonStep.kStep6){ 
                m_launcher.targetAndLaunch();
                if(m_intake.getMagazineSensor2() == false){
                    m_loopCount ++;
                }
                else if(m_loopCount >= RobotMap.AutonConstants.LOOPS_AFTER_LAUNCH){
                    //zeros the loop count to be used again
                    m_loopCount = 0;
                    m_step = AutonStep.kStep7;
                }
            }
            else if (m_step == AutonStep.kStep7){
                if (m_intake.getMagazineSensor1()){
                    m_secondBall = true;
                }
                if (m_intake.getMagazineSensor2() == false && m_secondBall == true){
                    m_intake.periodic();
                }
                else{
                    m_step = AutonStep.kStep8;
                }
            }
            else if (m_step == AutonStep.kStep8){ 
                m_launcher.targetAndLaunch();
                if(m_intake.getMagazineSensor2() == false){
                    m_loopCount ++;
                }
                else if(m_loopCount >= RobotMap.AutonConstants.LOOPS_AFTER_LAUNCH){
                    //zeros the loop count to be used again
                    m_loopCount = 0;
                    m_step = AutonStep.kStop;
                }
            }
            else if(m_step == AutonStep.kStop){
                System.out.println("Auton Completed");
                m_drivetrain.periodic(0, 0);
            }
        }
        m_sysOutCounter++;
    }

    /**
     * Method for driving forward a specified amount of inches at a specified
     * @param speed speed robot will travel at 
     * @param target target distance in inches. Should always be positive. In order to turn the other way, pass in a negative speed
     * @return whether or not we have reached our target
     */
    public boolean driveToTarget(double speed, double target){
        //Assigns encoder values to variables to be used in the logic of this method
        double rightEncoder = m_drivetrain.getRightDriveEncoderPosition();
        double leftEncoder = m_drivetrain.getLeftDriveEncoderPosition();
        //Translates target in inches to target in encoder ticks
        target = target * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;

        if(speed < 0){
            target = target * -1;
            if(target < 0){
                if(leftEncoder > target || rightEncoder > target){
                    m_drivetrain.periodic(speed,0);
                    return false;
                }
                else{
                    m_drivetrain.periodic(0,0);
                    m_drivetrain.zeroEncoders();
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
        }
        else if(speed > 0){
            if((target > 0) && (speed > 0)){
                if((leftEncoder < target) || (rightEncoder < target)){
                    m_drivetrain.periodic(speed, 0);
                    return false;
                }
                else{
                    m_drivetrain.periodic(0,0);
                    m_drivetrain.zeroEncoders();
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
        }
        else{
            System.out.println("Robot will never reach target; exiting pathing.");
            m_step = AutonStep.kStop;
            return false;
        }
    }

    /**
     * code taken from 2021 auton (protobranch)
     * @param speed speed at which we want to turn
     * @param target target angle in degrees we want to turn to
     * @return whether or not we have reached our target angle
     */
    public boolean turnToAngle(double speed, double target){
        double currentAngle = m_drivetrain.getGyro();
        //if target is on the left, this if statement will run
        if(speed < 0){
            target = target * -1;
            //Rotate Bound is a percentage (we are within a certain percentage of are target angle where we will be close enough)
            if(currentAngle < (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.periodic(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                return true;
            }
            else{
                m_drivetrain.periodic(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle);
                }
                return false;
            }
        }
        //if target is on the right, this if statement will run
        else{
            //Rotate Bound is a percentage (we are within a certain percentage of are target angle where we will be close enough)
            if(currentAngle > (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.periodic(0, 0);
                m_drivetrain.zeroEncoders();
                m_drivetrain.zeroGyro();
                return true;
            }
            else{
                m_drivetrain.periodic(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle);
                }
                return false;
            }
        }
    }

    //Method to set the auton path from the shuffleboard. 0 = Right Line, 1 = Right Wall, 2 = Left Wall
    private void selectPath(){
        if(m_currentAutonPath == 0){
            System.out.println("Setting Auton to Right Line Path");
            m_path = AutonPath.kRightLine;
        }
        else if(m_currentAutonPath == 1){
            System.out.println("Setting Auton to Right Wall Path");
            m_path = AutonPath.kRightWall;
        }
        else if(m_currentAutonPath == 2){
            System.out.println("Setting Auton to Left Wall Path");
            m_path = AutonPath.kLeftWall;
        }
        else{
            m_currentAutonPath = 0;
            System.out.println("Setting Auton to Right Line Path");
            m_path = AutonPath.kRightLine;
        }
    }
}