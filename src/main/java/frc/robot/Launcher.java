package frc.robot;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Launcher{
    //enum to store the state of the trajectory control solenoids
    public enum TrajectoryPosition{
        kUp ("Up"),

        kDown ("Down"),

        kUnkown ("Unknown");

        private String stateName;

        TrajectoryPosition (String stateName){
            this.stateName = stateName;
        }

        public String toString(){
            return this.stateName;

        }
    }
    //Declares limelight object
    private LimelightVision m_limelightVision;
    
    //Declares variables for the motors that move the launcher flywheel, the feeder wheel, and the turret angle
    //Not all of these motors will be TalonFXs, those are placeholders until we know what kinds of motors we'll be using
    private WPI_TalonFX m_masterFlywheelMotor;
    private WPI_TalonFX m_slaveFlywheelMotor;
    private WPI_VictorSPX m_feederMotor;
    private WPI_TalonSRX m_turretMotor;

    //Declares variables for the encoders
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_turretEncoder;


    //declares our double solenoid to be used on our trajectory control system
    private DoubleSolenoid m_solenoid;

    //declares state enum to track our current trajectory control state
    private TrajectoryPosition m_state;

    /**
     * Constructor for Launcher objects
     * @param limelightVision we pass in limelight to use in launch targeting
     */
    public Launcher(LimelightVision limelightVision){
        //Instantiates objects for the Launcher class
        m_limelightVision = limelightVision;

        m_masterFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new WPI_VictorSPX(RobotMap.LauncherConstants.FEEDER_MOTOR_ID);
        m_turretMotor = new WPI_TalonSRX(RobotMap.LauncherConstants.TURRET_MOTOR_ID);

        m_flywheelEncoder = new SensorCollection (m_masterFlywheelMotor);
        m_turretEncoder = new SensorCollection (m_turretMotor);

        m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.LauncherConstants.DOUBLESOLENOID_ANGLE_DOWN_PORT, RobotMap.LauncherConstants.DOUBLESOLENOID_ANGLE_UP_PORT);
    
        m_state = TrajectoryPosition.kUnkown;
    }

    /**
     * Initialization method for Launcher. Currently only zeros encoders and sets our default trajectory position to up
     */
    public void init(){
        zeroEncoders();
        setTrajectoryPosition(TrajectoryPosition.kUp);
        m_slaveFlywheelMotor.setInverted(true);
    }
    
    /**
     * Prepares launch sequence by turning turret towards the target and revving the launcher flywheel to the required speed
     */
    public void targetAndLaunch(){
        //Booleans to track when the two aspects of our launching sequence are ready
        boolean turretAngleReady = false;
        boolean flywheelMotorReady = false;
        //this if statement makes it so if we don't see a target, don't run the method and instead print "No Target Detected"
        if(m_limelightVision.seeTarget()){
            System.out.println("Target Detected");

            //checks if the turret encoder is within the tolerated range, and if we're not print a message and adjust
            if(getTurretPosition() > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && getTurretPosition() < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //this if statements checks to see if we are within the tolerated error range, and if we are set turret bool to true
                if(m_limelightVision.xAngleToTarget() < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && m_limelightVision.xAngleToTarget() > -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    setTurretSpeed(0);
                    turretAngleReady = true;
                }
                //if we are above the tolerated error range, turn the turret toward the tolerated error range
                else if(m_limelightVision.xAngleToTarget() > RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch and adjusts
                    System.out.println("Not Ready to Launch");
                    turretAngleReady = false;
                    setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
                }
                //if we are below the tolerated error range, turn the turret toward the tolerated error range
                else if(m_limelightVision.xAngleToTarget() < -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    //Prints out a message telling the driver that our robot is not yet ready to launch and adjusts
                    System.out.println("Not Ready to Launch");
                    turretAngleReady = false;
                    setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
                }
            }
            //If we are to the left of our motor limit, print out a message and turn right
            else if(getTurretPosition() < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch");
                turretAngleReady = false;
                setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }
            //If we are to the right of our motor limit, print out a message and turn left
            else if(getTurretPosition() > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch");
                turretAngleReady = false;
                setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }

            // We devide the distance in inches by a large number to get a reasonable value for our flywheel motor speed.
            // 100 is arbitrary and needs to be tested (more will probably need to be done so this is more fine tuned)
            double targetFlywheelSpeed = m_limelightVision.distToTarget(RobotMap.LimelightConstants.CAMERA_HEIGHT) / 100;
            setFlywheelSpeed(targetFlywheelSpeed);
            //Checks if our flywheel is at the target speed
            if(getRealSpeed() > targetFlywheelSpeed){
                flywheelMotorReady = true;
            }
            else{
                flywheelMotorReady = false;
            }

            //Prints out a message telling the driver when our robot is ready to launch and moves game pieces into the flywheel
            if(turretAngleReady == true && flywheelMotorReady == true){
                System.out.println("Commencing Launch Sequence");
                setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
            }
            else{
                setFeederSpeed(0);
            }
        }
        else {
            System.out.println("No Target Detected");
        }
    }

    /**
     * Shoots any game pieces out at a low speed to eject game pieces we don't want
     */
    public void expel(){
        setFlywheelSpeed(RobotMap.LauncherConstants.EXPEL_SPEED);
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }

    /**
     * sets the trajectory control system between up/down states
     * @param trajectoryPosition desired state (trajectoryPosition.kUp, trajectoryPosition.kDown)
     */
    public void setTrajectoryPosition(TrajectoryPosition trajectoryPosition){
        if (trajectoryPosition == m_state){
            return;
        }
        m_state = trajectoryPosition;

        if(m_state == TrajectoryPosition.kUp){
            setPiston(Value.kForward);
        }
        else if (m_state == TrajectoryPosition.kDown){
            setPiston(Value.kReverse);
        }
    }

    /**
     * @return current speed of flywheel motor
     */
    public double getRealSpeed(){
        return m_masterFlywheelMotor.getSelectedSensorVelocity();
    }

    /**
     * Sets the speed of the launcher flywheel motor
     * This method is public for manual turret testing zeroing motors in CopilotController
     * @param speed desired speed 
     */
    public void setFlywheelSpeed(double speed){
        m_masterFlywheelMotor.set(ControlMode.PercentOutput, speed);
        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
    }

    /**
     * Sets the speed of the motor that moves the turret
     * This is method is public for manual turret testing and zeroing motors in CopilotController
     * @param speed desired speed (Positive for one direction, negative for the other)
     */
    public void setTurretSpeed(double speed){
        m_turretMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the feeder motor
     * @param speed desired speed
     */
    public void setFeederSpeed(double speed){
        m_feederMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets pistons to a specific value
     * @param value Value.kForward, Value.kReverse
     */
    private void setPiston(DoubleSolenoid.Value value) {
        m_solenoid.set(value);
    }

    /**
     * Resets encoder values to 0
     */
    private void zeroEncoders(){
        m_flywheelEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);
        m_turretEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);
    }

    /**
     * @return the current encoder ticks on the turret motor
     */
    private double getTurretPosition(){
        return m_turretEncoder.getQuadraturePosition();
    }

    private void turretPIDConfig (){
        //Reverts all the configurations back to the factory default. This helps prevent unexpected behavior
        m_turretMotor.configFactoryDefault();
        //Configures Sensor Source for Primary PID. Selects using a primary close-loop
        m_turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, RobotMap.LauncherConstants.PID_LOOP_IDX, RobotMap.TIMEOUT_MS);
        //Configures the output deadband percentage for the turret motor.
        m_turretMotor.configNeutralDeadband(RobotMap.LauncherConstants.TURRET_MOTOR_DEADBAND, RobotMap.TIMEOUT_MS);

        //Configures the output and sensor direction for the Talon SRX
        m_turretMotor.setSensorPhase(false);
        m_turretMotor.setInverted(false);

        //Sets relecant frame periods to be at least as fast as periodic rate
        m_turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, RobotMap.TIMEOUT_MS);
        m_turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, RobotMap.TIMEOUT_MS);
    }

    /**
     * This is the preset for launching the ball 10 ft away into the low hub
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void lowPreset10(){
        setTrajectoryPosition(TrajectoryPosition.kDown);
        setFlywheelSpeed(0.7);
        if(getRealSpeed() >= 0.7){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 20 ft away into the low hub
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void lowPreset20(){
        setTrajectoryPosition(TrajectoryPosition.kDown);
        setFlywheelSpeed(0.9);
        if(getRealSpeed() >= 0.9){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 10 ft away into the high hub
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void highPreset10(){
        setTrajectoryPosition(TrajectoryPosition.kUp);
        setFlywheelSpeed(0.7);
        if(getRealSpeed() >= 0.7){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 20 ft away into the high hub
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void highPreset20(){
        setTrajectoryPosition(TrajectoryPosition.kUp);
        setFlywheelSpeed(0.9);
        if(getRealSpeed() >= 0.9){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }
}