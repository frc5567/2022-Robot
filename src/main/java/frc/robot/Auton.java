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

        kStop;
    }
    
    //declares instances of our drivetrain and the auton step enum
    private AutonStep m_step;
    private AutonPath m_path;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelightVision;
    boolean autonStartFlag = true;
    double targetEncoderTicks;
    double currentRightEncoderTicks;
    double currentLeftEncoderTicks;
    double targetGyro;
    double currentGyro;
    boolean canSeeTarget = false;
    double visionAngleToTarget;
    boolean m_limelightOff;
    double m_xToTarget;

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
        m_path = AutonPath.kLeftWall;
    }

    /**
     * this method will be run at the start of every auton period
     */
    public void init(){
        m_drivetrain.init();
        m_launcher.initLauncher();
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kLeftWall;
        m_drivetrain.shiftGear(Gear.kLowGear);
        // System.out.println("left encoder " + m_drivetrain.getLeftDriveEncoderPosition());
        // System.out.println(" Right encoder " + m_drivetrain.getRightDriveEncoderPosition());
        m_limelightVision.limelightInit();
        m_limelightOff = true;
    }

    /**
     * this method will be called many times a second during the auton period. currently all pseudo-code, need to create driveToTarget and turnToAngle methods 
     */
    public void periodic(){
        m_limelightVision.periodic();
        if(autonStartFlag){
            m_drivetrain.zeroEncoders();
            m_step = AutonStep.kStep1;
            System.out.println("STARTING AUTON");
            m_limelightOff = true;
            autonStartFlag = false;
        }
        currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
//        System.out.println("Right Encoder Ticks: " + currentRightEncoderTicks);
//        System.out.println("Left Encoder Ticks: " + currentLeftEncoderTicks);
        //m_drivetrain.zeroEncoders();
        if (m_path == AutonPath.kLeftWall){
            // System.out.println("Starting Auton. Path: " + m_path);
            // System.out.println("Starting Auton. Path: Left Wall");
            if(m_step == AutonStep.kStep1){
                // System.out.println("Current Step: " + m_step);
                // System.out.println("Current Step: 1");
                targetEncoderTicks = RobotMap.AutonConstants.STEP_ONE_TARGET * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;
                
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.STEP_ONE_TARGET)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                // System.out.println("Current Step: " + m_step);
                // System.out.println("Current Step: 2");
                currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
                currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
                targetGyro = -RobotMap.AutonConstants.STEP_TWO_TARGET * (1 - RobotMap.AutonConstants.ROTATE_BOUND);
                currentGyro = m_drivetrain.getGyro();
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.STEP_TWO_TARGET)){
                    m_drivetrain.arcadeDrive(0,0);
                    m_step = AutonStep.kStep3;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep3){
                if(m_limelightOff){
                    System.out.println("Turning on LEDS");
                    //NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
                    m_limelightVision.enableLEDs();
                    m_limelightOff = false;
                }
                m_xToTarget = m_limelightVision.xAngleToTarget();
                System.out.println(m_xToTarget);
                if(m_limelightVision.seeTarget()){
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("On Target");
                        m_drivetrain.arcadeDrive(0,0);
                         m_drivetrain.zeroEncoders();
                         m_limelightVision.disableLEDs();
                         m_limelightOff = true;
                         m_step = AutonStep.kStop;
                    }
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("Not On Target");
                        m_drivetrain.arcadeDrive(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        System.out.println("Not On Target");
                        m_drivetrain.arcadeDrive(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    System.out.println("No Target Detected");
                }
            }
            else if(m_step == AutonStep.kStep4){

                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 4");
                if(canSeeTarget){
                    System.out.println("Target Detected!!!!!!!!!!!!");
                    m_step = AutonStep.kStop;
                }
                else{
                    System.out.println("No Target Detected!!!!!!!!!!!!");
                    canSeeTarget = m_limelightVision.seeTarget();
                }
            }
            else if(m_step == AutonStep.kStep5){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 5");
                turnToAngle(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FULL_TURN);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 6");
                if(m_launcher.checkLaunchSensor()){
                    m_step = AutonStep.kStop;
                }
                else{
                    m_launcher.launch();
                }
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightWall){
            if(m_step == AutonStep.kStep1){
                System.out.println("Current Step: " + m_step);
                driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_ANGLE_COUNTERCLOCKWISE);
                m_step = AutonStep.kStep3;
            }
            else if(m_step == AutonStep.kStep3){
                System.out.println("Current Step: " + m_step);
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_step = AutonStep.kStep4;
            }
            else if(m_step == AutonStep.kStep4){
                System.out.println("Current Step: " + m_step);
                if(m_intake.checkMagazineSensor()){
                    m_step = AutonStep.kStep5;
                }
                else{
                    m_drivetrain.arcadeDrive(1, 0);
                    m_intake.takeIn();
                }
            }
            else if(m_step == AutonStep.kStep5){
                System.out.println("Current Step: " + m_step);
                driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FULL_TURN);
                m_step = AutonStep.kStep7;
            }
            else if(m_step == AutonStep.kStep7){
                System.out.println("Current Step: " + m_step);
                if(m_launcher.checkLaunchSensor()){
                    m_step = AutonStep.kStop;
                }
                else{
                    m_launcher.launch();
                }
            }
            else if(m_step == AutonStep.kStop){
                System.out.println("Auton completed");
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightLine){ 
            //drive forward a much smaller amount than other paths
            if(m_step == AutonStep.kStep1){
                System.out.println("Current Step: " + m_step);
                driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_ANGLE_CLOCKWISE);
                m_step = AutonStep.kStep3;
            }
            else if(m_step == AutonStep.kStep3){
                System.out.println("Current Step: " + m_step);
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_step = AutonStep.kStep4;
            }
            else if(m_step == AutonStep.kStep4){
                System.out.println("Current Step: " + m_step);
                if(m_intake.checkMagazineSensor()){
                    m_step = AutonStep.kStep5;
                }
                else{
                    m_drivetrain.arcadeDrive(1, 0);
                    m_intake.takeIn();
                }
            }
            else if(m_step == AutonStep.kStep5){
                System.out.println("Current Step: " + m_step);
                driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FULL_TURN);
                m_step = AutonStep.kStep7;
            }
            else if(m_step == AutonStep.kStep7){
                System.out.println("Current Step: " + m_step);
                if(m_launcher.checkLaunchSensor()){
                    m_step = AutonStep.kStop;
                }
                else{
                    m_launcher.launch();
                }
            }
            else if(m_step == AutonStep.kStop){
                System.out.println("Auton Completed");
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
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
        System.out.println("EncoderTarget: " + target);
        System.out.println("Right Encoder Ticks: " + rightEncoder);
        System.out.println("Left Encoder Ticks: " + leftEncoder);

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
            System.out.println("Target Angle: " + target);
            System.out.println("Current Angle " + currentAngle);
            if(currentAngle < (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.arcadeDrive(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                System.out.println("Zero Gyro " + m_drivetrain.getGyro()); 
                System.out.println("At Target Angle " + currentAngle);
                return true;
            }
            else{
                m_drivetrain.arcadeDrive(0, speed);
                System.out.println("Not At Target Angle " + currentAngle);
                return false;
            }
        }
        //if target is on the right, this if statement will run
        else{
            System.out.println("Target Angle: " + target);
            System.out.println("Current Angle: " + currentAngle);
            if(currentAngle > (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.arcadeDrive(0, 0);
                m_drivetrain.zeroEncoders();
                m_drivetrain.zeroGyro();
                System.out.println("zero Gyro" + m_drivetrain.getGyro());
                System.out.println("At Target Angle " + currentAngle);
                return true;
            }
            else{
                m_drivetrain.arcadeDrive(0, speed);
                System.out.println("Not At Target Angle " + currentAngle);
                return false;
            }
        }
    }
}