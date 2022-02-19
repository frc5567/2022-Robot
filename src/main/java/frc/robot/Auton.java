package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Drivetrain.Gear;
import frc.robot.Intake.IntakeState;


public class Auton{    
    //enum for what path we are going to take in auton
    public enum AutonPath{
        //auton path for starting on the hub wall on the left side
        kLeftWall,

        //auton path for starting on the hub wall on the left side
        kRightWall,

        //auton path for starting on the edge of the tarmac line on the right side
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

        kStop;
    }
    
    //declares variables for the auton class
    private AutonStep m_step;
    private AutonPath m_path;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelightVision;
    boolean m_autonStartFlag = true;
    double m_targetEncoderTicks;
    double m_currentRightEncoderTicks;
    double m_currentLeftEncoderTicks;
    double m_targetGyro;
    double m_currentGyro;
    boolean m_canSeeTarget = false;
    double m_visionAngleToTarget;
    boolean m_limelightOff;
    double m_xToTarget;
    int m_sysOutCounter;
    boolean m_doSysOut;

    /**
     * constructor for auton
     * @param drivetrain drivetrain mechanism on the robot
     */
    public Auton(Drivetrain drivetrain, Launcher launcher, Intake intake, LimelightVision limelightVision){
        m_drivetrain = drivetrain;
        m_launcher = launcher;
        m_intake = intake;
        m_limelightVision = limelightVision;

        m_step = AutonStep.kStep1;
        m_path = AutonPath.kRightWall;
    }

    /**
     * this method will be run at the start of every auton period
     */
    public void init(){
        m_drivetrain.init();
        m_launcher.initLauncher();
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kRightWall;
        m_drivetrain.shiftGear(Gear.kLowGear);
        // System.out.println("left encoder " + m_drivetrain.getLeftDriveEncoderPosition());
        // System.out.println(" Right encoder " + m_drivetrain.getRightDriveEncoderPosition());
        m_limelightVision.limelightInit();
        m_doSysOut = true;
    }

    /**
     * this method will be called many times a second during the auton period. currently all pseudo-code, need to create driveToTarget and turnToAngle methods 
     */
    public void periodic(){
        m_limelightVision.periodic();
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

        if(m_autonStartFlag){
            m_drivetrain.zeroEncoders();
            m_step = AutonStep.kStep1;
            System.out.println("STARTING AUTON");
            m_autonStartFlag = false;
        }
        m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
        //System.out.println("Right Encoder Ticks: " + currentRightEncoderTicks);
        //System.out.println("Left Encoder Ticks: " + currentLeftEncoderTicks);

        //Counter for sysouts
        if((m_sysOutCounter % 50) == 0){
            m_doSysOut = true;
        }
        else{
            m_doSysOut = false;
        }

        //m_drivetrain.zeroEncoders();
        if (m_path == AutonPath.kLeftWall){
            if(m_step == AutonStep.kStep1){
                m_targetEncoderTicks = RobotMap.AutonConstants.LEFT_WALL_STEP_ONE_TARGET_DISTANCE * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_ONE_TARGET_DISTANCE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                //targetGyro = -RobotMap.AutonConstants.LEFT_WALL_STEP_TWO_TARGET_ANGLE * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                //currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_TWO_TARGET_ANGLE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep3){
                //m_intake.setIntakeExtension(IntakeState.kExtended);
                m_step = AutonStep.kStep4;
            }
            if(m_step == AutonStep.kStep4){
                m_targetEncoderTicks = RobotMap.AutonConstants.LEFT_WALL_STEP_FOUR_TARGET_DISTANCE * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_FOUR_TARGET_DISTANCE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep5){
                //m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_step = AutonStep.kStep6;
            }

                //TODO: correct step numbers
            else if(m_step == AutonStep.kStep6){
                //System.out.println(m_drivetrain.getGyro());
                m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                //targetGyro = RobotMap.AutonConstants.LEFT_WALL_STEP_SIX_TARGET_ANGLE * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                //currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.LEFT_WALL_STEP_SIX_TARGET_ANGLE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep7;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                    m_limelightVision.enableLEDs();
                }
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                if(m_limelightVision.seeTarget()){
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.arcadeDrive(0,0);
                         m_drivetrain.zeroEncoders();
                         m_limelightVision.disableLEDs();
                         m_step = AutonStep.kStep8;
                    }
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.arcadeDrive(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                        }
                        m_drivetrain.arcadeDrive(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            else if(m_step == AutonStep.kStep8){
                //this is the step where we will launch 
                System.out.println("Launch!");
                m_step = AutonStep.kStop;
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightWall){
            if(m_step == AutonStep.kStep1){
                m_targetEncoderTicks = RobotMap.AutonConstants.RIGHT_WALL_STEP_ONE_TARGET_DISTANCE * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_ONE_TARGET_DISTANCE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                //targetGyro = -RobotMap.AutonConstants.RIGHT_WALL_STEP_TWO_TARGET_ANGLE * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                //currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_TWO_TARGET_ANGLE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep3){
                //m_intake.setIntakeExtension(IntakeState.kExtended);
                m_step = AutonStep.kStep4;
            }
            if(m_step == AutonStep.kStep4){
                m_targetEncoderTicks = RobotMap.AutonConstants.RIGHT_WALL_STEP_FOUR_TARGET_DISTANCE * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_FOUR_TARGET_DISTANCE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            if(m_step == AutonStep.kStep5){
                //m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_step = AutonStep.kStep6;
            }

                //TODO: correct step numbers
            else if(m_step == AutonStep.kStep6){
                //System.out.println( m_drivetrain.getGyro());
                m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                //targetGyro = RobotMap.AutonConstants.RIGHT_WALL_STEP_SIX_TARGET_ANGLE * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                //currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_WALL_STEP_SIX_TARGET_ANGLE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep7;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                    m_limelightVision.enableLEDs();
                }
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                if(m_limelightVision.seeTarget()){
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.arcadeDrive(0,0);
                         m_drivetrain.zeroEncoders();
                         m_limelightVision.disableLEDs();
                         m_step = AutonStep.kStep8;
                    }
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.arcadeDrive(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.arcadeDrive(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                            System.out.println("No target detected");    
                        }
                }
            }
            else if(m_step == AutonStep.kStep8){
                //this is the step where we will launch 
                System.out.println("Launch!");
                m_step = AutonStep.kStop;
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightLine){ 
            //drive forward a much smaller amount than other paths
            if(m_step == AutonStep.kStep1){
                //m_intake.setIntakeExtension(IntakeState.kExtended);
                System.out.println("Intake Extended");
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                m_targetEncoderTicks = RobotMap.AutonConstants.RIGHT_LINE_STEP_TWO_TARGET_DISTANCE * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                m_intake.takeIn();
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.RIGHT_LINE_STEP_TWO_TARGET_DISTANCE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep3){
                //m_intake.setIntakeExtension(IntakeState.kRetracted);
                System.out.println("Intake Retracted");
                m_step = AutonStep.kStep4;
            }
            else if(m_step == AutonStep.kStep4){
                //System.out.println(m_drivetrain.getGyro());
                m_currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                m_currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                //targetGyro = RobotMap.AutonConstants.LEFT_WALL_STEP_SIX_TARGET_ANGLE * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                //currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.RIGHT_LINE_STEP_FOUR_TARGET_ANGLE)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep5;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep5){
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                    m_limelightVision.enableLEDs();
                }
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                if(m_limelightVision.seeTarget()){
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.arcadeDrive(0,0);
                         m_drivetrain.zeroEncoders();
                         m_limelightVision.disableLEDs();
                         m_step = AutonStep.kStep6;
                    }
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");    
                        }
                        m_drivetrain.arcadeDrive(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                        }
                        m_drivetrain.arcadeDrive(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            else if(m_step == AutonStep.kStep6){
                //this is the step where we will launch 
                System.out.println("Launch!");
                m_step = AutonStep.kStop;
            }
            else if(m_step == AutonStep.kStop){
                System.out.println("Auton Completed");
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        m_sysOutCounter++;
    }

    /**
     * code taken from 2021 auton (protobranch)
     * @param speed speed robot will travel at 
     * @param target target distance in inches
     * @return whether or not we have reached our target
     */
    public boolean driveToTarget(double speed, double target){
        double rightEncoder = m_drivetrain.getRightDriveEncoderPosition();
        double leftEncoder = m_drivetrain.getLeftDriveEncoderPosition();
        target = target * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
        //System.out.println("EncoderTarget: " + target);
        //System.out.println("Right Encoder Ticks: " + rightEncoder);
        //System.out.println("Left Encoder Ticks: " + leftEncoder);

        if(speed < 0){
            target = target * -1;
            if((target > 0) && (speed > 0)){
                if((leftEncoder < target) || (rightEncoder < target)){
                    //m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.arcadeDrive(speed, 0);
                    return false;
                }
                else{
                    m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.zeroEncoders();
                    //TODO: look for docuemtation about using gyros
                    return true;
                }
            }
            else if((target < 0) && (speed < 0)){
                if(leftEncoder > target || rightEncoder > target){
                    //m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.arcadeDrive(speed,0);
                    return false;
                }
                else{
                    m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.zeroEncoders();
                    //gyro again
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
        }
        //if speed is greater than 0
        else{
            if((target > 0) && (speed > 0)){
                if((leftEncoder < target) || (rightEncoder < target)){
                    //m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.arcadeDrive(speed, 0);
                    return false;
                }
                else{
                    m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.zeroEncoders();
                    //gyro again
                    return true;
                }
            }
            else if((target < 0) && (speed < 0)){
                if((leftEncoder > target) || (rightEncoder > target)){
                    //m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.arcadeDrive(speed,0);
                    return false;
                }
                else{
                    m_drivetrain.arcadeDrive(0,0);
                    m_drivetrain.zeroEncoders();
                    //gyro again
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
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
            //System.out.println("Target Angle: " + target);
            //System.out.println("Current Angle " + currentAngle);
            if(currentAngle < (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.arcadeDrive(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                //System.out.println("Zero Gyro " + m_drivetrain.getGyro()); 
                //System.out.println("At Target Angle " + currentAngle);
                return true;
            }
            else{
                m_drivetrain.arcadeDrive(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle);
                }
                return false;
            }
        }
        //if target is on the right, this if statement will run
        else{
            //System.out.println("Target Angle: " + target);
            //System.out.println("Current Angle: " + currentAngle);
            if(currentAngle > (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.arcadeDrive(0, 0);
                m_drivetrain.zeroEncoders();
                m_drivetrain.zeroGyro();
                //System.out.println("zero Gyro" + m_drivetrain.getGyro());
                //System.out.println("At Target Angle " + currentAngle);
                return true;
            }
            else{
                m_drivetrain.arcadeDrive(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle);
                }
                return false;
            }
        }
    }
}