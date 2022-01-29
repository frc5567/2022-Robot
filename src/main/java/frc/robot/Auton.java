package frc.robot;

import javax.naming.ldap.LdapReferralException;

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
    public Auton(Drivetrain drivetrain, Launcher launcher, Intake intake){
        m_drivetrain = drivetrain;
        m_launcher = launcher;
        m_intake = intake;
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kLeftWall;
    }

    //this method will be run at the start of every auton period
    public void init(){
        m_drivetrain.zeroEncoders();
        m_launcher.zeroEncoders();
        m_step = AutonStep.kStep1;
    }

    //this method will be called many times a second during the auton period. currently all pseudo-code, need to create driveToTarget and turnToAngle methods 
    public void periodic(){
        if (m_path == AutonPath.kLeftWall){
            /*
            if(m_step == AutonStep.kStep1){

                if(driveToTarget(Speed, Distance)){
                    m_step = AutonStep.kStep2;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                
                if(turnToAngle(Clockwise Speed, target angle)){
                    m_step = AutonStep.kStep3;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep3){

                if(toggleIntakeExtension(m_intake.kExtended)){
                    m_step = AutonStep.kStep4;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep4){

                if(turn on intake)){
                    m_step = AutonStep.kStep5;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep5){

                if(driveToTarget(Speed, Distance)){
                    m_step = AutonStep.kStep6;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep6){

                if(turnToAngle(Clockwise Speed, 180)){
                    m_step = AutonStep.kStep7;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){

                if(target and launch both balls)){
                    m_step = AutonStep.kStop;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.arcadeDrive(0, 0);
            }
            */

        }
        else if (m_path == AutonPath.kRightWall){
            /*
            -drive forward
            -turn slightly left
            -drive forward to ball
            -intake ball
            -turn about 180 degress back towards the hub
            -target and shoot both the pre-loaded ball and the picked-up ball
            */
        }
        else if (m_path == AutonPath.kRightLine){
            /*  
            -drive forward a much smaller amount
            -turn slightly right
            -drive forward to ball
            -intake ball
            -turn about 180 degress back towards the hub
            -target and shoot both the pre-loaded ball and the picked-up ball
            */
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
     * code taken from 2021 auton (protobranch) with prints commented out.
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