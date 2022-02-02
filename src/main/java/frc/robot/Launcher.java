package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
//Import for sensor 
import edu.wpi.first.wpilibj.DigitalInput;

public class Launcher{
    //Declares intake to use magazine motors in launch method
    private Intake m_intake;

    //Declares limelight object
    private LimelightVision m_limelightVision;
    
    //Declares variables for the motors that move the launcher flywheel, the feeder wheel, the turret angle, and the trajectory control
    //Not all of these motors will be TalonFXs, those are placeholders until we know what kinds of motors we'll be using
    private WPI_TalonFX m_masterFlywheelMotor;
    private WPI_TalonFX m_slaveFlywheelMotor;
    private WPI_TalonFX m_feederMotor;
    private WPI_TalonFX m_turretMotor;
    private WPI_TalonFX m_trajectoryMotor;

    //Declares variables for the encoders
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_feederEncoder;
    private SensorCollection m_turretEncoder;
    private SensorCollection m_trajectoryEncoder;

    private DigitalInput m_launchSensor;

    /**
     * Constructor for Launcher objects
     */
    public Launcher(){
        m_intake = new Intake();

        m_masterFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new WPI_TalonFX(RobotMap.LauncherConstants.FEEDER_FALCON_ID);
        m_turretMotor = new WPI_TalonFX(RobotMap.LauncherConstants.TURRET_FALCON_ID);

        m_limelightVision = new LimelightVision();

        m_trajectoryEncoder = new SensorCollection (m_trajectoryMotor);

        m_launchSensor = new DigitalInput(RobotMap.LauncherConstants.LAUNCH_SENSOR_PORT);
    }
    
    /**
     * Prepares launch sequence by turning turret towards the target, moving the trajectory controller to the required angle, and revving the launcher flywheel to the required speed
     */
    public void launch(){
        boolean turretAngleReady = false;
        boolean trajectoryReady = false;
        boolean flywheelMotorReady = false;
        //this if statement makes it so if we don't see a target, don't run the method and instead print "No Target Detected"
        if(m_limelightVision.seeTarget()){
            System.out.println("Target Detected");

            //checks if the turret encoder is within the tolerated range, and if we're not print a message and adjust
            if(getTurretPosition() > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && getTurretPosition() < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //this if statements checks to see if we are within the tolerated error range, and if we are set turret bool to true
                if(m_limelightVision.xAngleToTarget() < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && m_limelightVision.xAngleToTarget() > -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    turretAngleReady = true;
                }
                //if we are above the tolerated error range, turn the turret toward the tolerated error range
                else if(m_limelightVision.xAngleToTarget() > RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch
                    System.out.println("Not Ready to Launch");
                    turretAngleReady = false;
                    setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
                }
                //if we are below the tolerated error range, turn the turret toward the tolerated error range
                else if(m_limelightVision.xAngleToTarget() < -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch
                    System.out.println("Not Ready to Launch");
                    turretAngleReady = false;
                    setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
                }
            }
            //If we are to the left of our motor limit, print out a message and turn right
            else if(getTurretPosition() < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch");
                turretAngleReady = false;
                setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
            }
            //If we are to the right of our motor limit, print out a message and turn left
            else if(getTurretPosition() > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch");
                turretAngleReady = false;
                setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
            }

            //variable to find the target position of the trajectory control motor
            double targetTrajectoryPosition = (-(m_limelightVision.yAngleToTarget()/2) + 0.5) * RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT;

            //checks if the encoder is within the range we want, for now 0 and 30000 but needs testing
            if(getTrajectoryPosition() > 0 && getTrajectoryPosition() < RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT){
                //checks to see if we are within the tolerated error range
                if(Math.abs(getTrajectoryPosition() - targetTrajectoryPosition) <= RobotMap.LauncherConstants.TOLERATED_TRAJECTORY_ERROR){
                    trajectoryReady = true;
                }
                //move the trajectory control motor toward the target position
                else if(getTrajectoryPosition() < targetTrajectoryPosition){
                    System.out.println("Not Ready to Launch");
                    trajectoryReady = false;
                    setTrajectorySpeed(RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                }
                else if(getTrajectoryPosition() > targetTrajectoryPosition){
                    System.out.println("Not Ready to Launch");
                    trajectoryReady = false;
                    setTrajectorySpeed(-RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                }
            }
            else if(getTrajectoryPosition() > RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch");
                trajectoryReady = false;
                setTrajectorySpeed(-RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
            }
            else if(getTrajectoryPosition() < 0){
                System.out.println("Not Ready to Launch");
                trajectoryReady = false;
                setTrajectorySpeed(RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
            }

            // We devide the distance in inches by a large number to get a reasonable value for our flywheel motor speed.
            // 100 is arbitrary and needs to be tested (more will probably need to be done so this is more fine tuned)
            double targetFlywheelSpeed = m_limelightVision.distToTarget(RobotMap.LimelightConstants.CAMERA_HEIGHT) / 100;
            setFlywheelSpeed(targetFlywheelSpeed);
            //Checks if our flywheel is at the target speed
            if(getRealSpeed() == targetFlywheelSpeed){
                flywheelMotorReady = true;
            }
            else{
                flywheelMotorReady = false;
            }

            //Prints out a message telling the driver when our robot is ready to launch
            if(turretAngleReady == true && trajectoryReady == true && flywheelMotorReady == true){
                System.out.println("Commencing Launch Sequence");
                advanceBalls();
            }
        }
        else {
            System.out.println("No Target Detected");
        }
    }

    /**
     * feeds the ball into the flywheel and advances the magazine
     */
    private void advanceBalls(){
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
    private void setFeederSpeed(double speed){
        m_feederMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the launcher flywheel motor
     * @param speed desired speed
     */
    private void setFlywheelSpeed(double speed){
        m_masterFlywheelMotor.set(ControlMode.PercentOutput, speed);
        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
    }

    /**
     * Sets the speed of the motor that moves the turret
     * @param speed desired speed
     */
    private void setTurretSpeed(double speed){
        m_turretMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the motor that moves the trajectory control
     * @param speed desired speed
     */
    private void setTrajectorySpeed(double speed){
        m_trajectoryMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * @return current speed of flywheel motor
     */
    public double getRealSpeed(){
        return m_masterFlywheelMotor.getSelectedSensorVelocity();
    }

    /**
     * @return whether or not the launch snesor is being activated
     */
    public boolean checkLaunchSensor(){
        return m_launchSensor.get();
    }
}