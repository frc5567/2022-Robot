package frc.robot;

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

    /**
     * constructor for auton
     * @param drivetrain drivetrain mechanism on the robot
     */
    public Auton(){
        m_drivetrain = new Drivetrain();
        m_launcher = new Launcher();
        m_intake = new Intake();
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kLeftWall;
    }

    /**
     * this method will be run at the start of every auton period
     */
    public void init(){
        m_drivetrain.zeroEncoders();
        m_launcher.zeroEncoders();
        m_step = AutonStep.kStep1;
    }

    /**
     * this method will be called many times a second during the auton period. currently all pseudo-code, need to create driveToTarget and turnToAngle methods 
     */
    public void periodic(){
        if (m_path == AutonPath.kLeftWall){
            System.out.println("Starting Auton. Path: " + m_path);
            System.out.println("Starting Auton. Path: Left Wall");
            if(m_step == AutonStep.kStep1){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 1");
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, 12);
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 2");
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, -30);
                m_step = AutonStep.kStep3;
            }
            else if(m_step == AutonStep.kStep3){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 3");
                System.out.println("Extend Intake");
            }
            else if(m_step == AutonStep.kStep4){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 4");
                System.out.println("Drive forward until intake picks up ball");
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, 12);
            }
            else if(m_step == AutonStep.kStep5){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 5");
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.FULL_TURN);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                System.out.println("Current Step: 6");
                System.out.println("Launch both balls");
            }
            else if(m_step == AutonStep.kStop){
                System.out.println("Current Step: " + m_step);
                System.out.println("Auton completed");
                m_drivetrain.arcadeDrive(0, 0);
            }
        }
        else if (m_path == AutonPath.kRightWall){
            if(m_step == AutonStep.kStep1){
                System.out.println("Current Step: " + m_step);
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_ANGLE_COUNTERCLOCKWISE);
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
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.FULL_TURN);
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
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep2;
            }
            else if(m_step == AutonStep.kStep2){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_ANGLE_CLOCKWISE);
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
                driveToTarget(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.PLACEHOLDER_VALUE_DISTANCE);
                m_step = AutonStep.kStep6;
            }
            else if(m_step == AutonStep.kStep6){
                System.out.println("Current Step: " + m_step);
                turnToAngle(RobotMap.AutonConstants.PLACEHOLDER_VALUE_SPEED, RobotMap.AutonConstants.FULL_TURN);
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
        //if speed is greater than 0 (or 0)
        else{
            if((target > 0) && (speed > 0)){
                if((leftEncoder < target) || (rightEncoder < target)){
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
        float currentAngle = m_drivetrain.getGyro();
        //if target is on the left, this if statement will run
        if(speed <0){
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