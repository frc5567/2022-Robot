package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
//import servo
import edu.wpi.first.wpilibj.Servo;

public class Launcher{
    //Declares intake to use magazine motors in launch method
    private Intake m_intake;

    //Declares limelight object
    private LimelightVision m_limelightVision;
    
    //Declares variables for the motor that moves the launcher flywheel and the motor that controls the turret angle
    private TalonFX m_masterFlywheelMotor;
    private TalonFX m_slaveFlywheelMotor;
    private TalonFX m_feederMotor;
    private TalonFX m_turretMotor;
    private TalonFX m_trajectoryMotor;
    Servo m_trajectoryServo;

    //Declares variables for the encoders
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_feederEncoder;
    private SensorCollection m_turretEncoder;
    private SensorCollection m_trajectoryEncoder;

    /**
     * Constructor for Launcher objects
     */
    public Launcher(){
        m_intake = new Intake();

        m_masterFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new TalonFX(RobotMap.LauncherConstants.FEEDER_FALCON_ID);
        m_turretMotor = new TalonFX(RobotMap.LauncherConstants.TURRET_FALCON_ID);
        m_trajectoryServo = new Servo(0);

        m_limelightVision = new LimelightVision();

        m_trajectoryEncoder = new SensorCollection (m_trajectoryMotor);
    }
    
    /**
     * Prepares launch sequence by turning turret towards the target, moving the trajectory controller to the required angle, and revving the launcher flywheel to the required speed
     */
    public void prepareLaunch(){
        //this if statement makes it so if we don't see a target, don't run the method and instead print "No Target Detected"
        if(m_limelightVision.seeTarget()){
            System.out.println("Target Detected");

            //checks if the turret encoder is within the range we want, for now -75000 and 75000 but needs testing
            if(getTurretPosition() > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && getTurretPosition() < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //this if statements checks to see if we are within the tolerated error range
                if(m_limelightVision.xAngleToTarget() > RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR || m_limelightVision.xAngleToTarget() < -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch
                    System.out.println("Not Ready to Launch");
    
                    //if we are above the tolerated error range, turn the turret toward the tolerated error range
                    if(m_limelightVision.xAngleToTarget() > RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                        setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
                    }
                    //if we are below the tolerated error range, turn the turret toward the tolerated error range
                    else if(m_limelightVision.xAngleToTarget() < -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                        setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
                    }
                }
            }
            else if(getTurretPosition() < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
            }
            else if(getTurretPosition() > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
            }

            //variable to find the target position of the trajectory control motor
            double targetTrajectoryPosition = (-(m_limelightVision.yAngleToTarget()/2) + 0.5) * RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT;

            //checks if the encoder is within the range we want, for now 0 and 30000 but needs testing
            if(getTrajectoryPosition() > 0 && getTrajectoryPosition() < RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT){
                //checks to see if we are within the tolerated error range
                if(Math.abs(getTrajectoryPosition() - targetTrajectoryPosition) >= RobotMap.LauncherConstants.TOLERATED_TRAJECTORY_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch
                    System.out.println("Not Ready to Launch");

                    //move the trajectory control motor toward the target position
                    if(getTrajectoryPosition() < targetTrajectoryPosition){
                        setTrajectorySpeed(RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                    }
                    else if(getTrajectoryPosition() > targetTrajectoryPosition){
                        setTrajectorySpeed(-RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                    }
                }
            }
            else if(getTrajectoryPosition() > RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT){
                setTrajectorySpeed(-RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
            }
            else if(getTrajectoryPosition() < 0){
                setTrajectorySpeed(RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
            }

            // We devide the distance in inches by a large number to get a reasonable value for our flywheel motor speed.
            // 100 is arbitrary and needs to be tested (more will probably need to be done so this is more fine tuned)
            double targetFlywheelSpeed = m_limelightVision.distToTarget(RobotMap.LimelightConstants.CAMERA_HEIGHT) / 100;
            setFlywheelSpeed(targetFlywheelSpeed);

            //Prints out a message telling the driver when our robot is ready to launch
            if(m_limelightVision.xAngleToTarget() < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && m_limelightVision.xAngleToTarget() > -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && Math.abs(getTrajectoryPosition() - targetTrajectoryPosition) <= RobotMap.LauncherConstants.TOLERATED_TRAJECTORY_ERROR && getRealSpeed() == targetFlywheelSpeed){
                System.out.println("Ready to Launch");
            }
        }
        else {
            System.out.println("No Target Detected");
        }
    }

    /*
    This is a seperate version of the method in case we end up using a servo for trajectory control
    public void prepareLaunch(){
        if(m_limelightVision.seeTarget()){
            if(m_limelightVision.xAngleToTarget() != 0){
                if(m_limelightVision.xAngleToTarget() > 0){
                    setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
                }
                else if(m_limelightVision.xAngleToTarget() < 0){
                    setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
                }
            }
            
            //This will be inaccurate and will need to be enhanced to be more specific later, ideally with PID
            m_trajectoryServo.set(-(m_limelightVision.yAngleToTarget()/2) + 0.5);

            // We devide the distance in inches by a large number to get a reasonable value for our flywheel motor speed.
            // 100 is arbitrary and needs to be tested (more will probably need to be done so this is more fine tuned)
            setFlywheelSpeed(m_limelightVision.distToTarget(RobotMap.LimelightConstants.CAMERA_HEIGHT) / 100);
        }
    }
    */

    /**
     * feeds the ball into the flywheel and advances the magazine
     */
    public void launch(){
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
        
    }

    /**
     * Shoots any balls out at a low speed to eject balls we don't want
     */
    public void expel(){
        setFlywheelSpeed(RobotMap.LauncherConstants.EXPEL_SPEED);
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }

    /**
     * Zeros encoders
     */
    public void zeroEncoders(){
        m_flywheelEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_feederEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_turretEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_trajectoryEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
    }

    /**
     * @return the current encoder ticks on the trajectory control motor
     */
    public double getTrajectoryPosition(){
        return m_trajectoryEncoder.getQuadraturePosition();
    }

    /**
     * @return the current encoder ticks on the turret motor
     */
    public double getTurretPosition(){
        return m_turretEncoder.getQuadraturePosition();
    }

    /**
     * Sets the speed of the feeder motor
     * @param speed desired speed
     */
    public void setFeederSpeed(double speed){
        m_feederMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the launcher flywheel motor
     * @param speed desired speed
     */
    public void setFlywheelSpeed(double speed){
        m_masterFlywheelMotor.set(ControlMode.PercentOutput, speed);
        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
    }

    /**
     * Sets the speed of the motor that moves the turret
     * @param speed desired speed
     */
    public void setTurretSpeed(double speed){
        m_turretMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the motor that moves the trajectory control
     * @param speed desired speed
     */
    public void setTrajectorySpeed(double speed){
        m_trajectoryMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * @return current speed of flywheel motor
     */
    public double getRealSpeed(){
        return m_masterFlywheelMotor.getSelectedSensorVelocity();
    }
}