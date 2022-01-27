package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
//import servo
import edu.wpi.first.wpilibj.Servo;

public class Launcher{
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
        m_masterFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new TalonFX(RobotMap.LauncherConstants.FEEDER_FALCON_ID);
        m_turretMotor = new TalonFX(RobotMap.LauncherConstants.TURRET_FALCON_ID);
        m_trajectoryServo = new Servo(0);

        m_limelightVision = new LimelightVision();

        m_trajectoryEncoder = new SensorCollection (m_trajectoryMotor);
    }
    
    
    //method for preparing the launch sequence
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
            if(getTrajectoryPosition() > 0 && getTrajectoryPosition() < RobotMap.LauncherConstants.TRAJECTORY_ENCODER_LIMIT){
                if(m_limelightVision.yAngleToTarget() > 0){
                    setTrajectorySpeed(-RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                }
                else if(m_limelightVision.yAngleToTarget() < 0){
                    setTrajectorySpeed(RobotMap.LauncherConstants.TRAJECTORY_MOTOR_SPEED);
                }
            }

            // We devide the distance in inches by a large number to get a reasonable value for our flywheel motor speed.
            // 100 is arbitrary and needs to be tested (more will probably need to be done so this is more fine tuned)
            setFlywheelSpeed(m_limelightVision.distToTarget(RobotMap.LimelightConstants.CAMERA_HEIGHT) / 100);
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

    //method for feeding the ball into the flywheel once it's revved up to speed
    public void launch(){
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }

    //method for getting rid of balls we don't want
    public void expel(){
        setFlywheelSpeed(RobotMap.LauncherConstants.EXPEL_SPEED);
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }

    //Zeros encoders
    public void zeroEncoders(){
        m_flywheelEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_feederEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_turretEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_trajectoryEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
    }

    public double getTrajectoryPosition(){
        return m_trajectoryEncoder.getQuadraturePosition();
    }

    //Sets the speed of the feeder motor
    public void setFeederSpeed(double speed){
        m_feederMotor.set(ControlMode.PercentOutput, speed);
    }

    //Sets the speed of the launcher flywheel motor
    public void setFlywheelSpeed(double speed){
        m_masterFlywheelMotor.set(ControlMode.PercentOutput, speed);
        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
    }

    public void setTurretSpeed(double speed){
        m_turretMotor.set(ControlMode.PercentOutput, speed);
    }

    public void setTrajectorySpeed(double speed){
        m_trajectoryMotor.set(ControlMode.PercentOutput, speed);
    }

    //Returns current speed of flywheel motor
    public double getRealSpeed(){
        return m_masterFlywheelMotor.getSelectedSensorVelocity();
    }
}