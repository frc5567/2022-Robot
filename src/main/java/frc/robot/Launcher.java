package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
    private Drivetrain m_drivetrain;
    
    //Declares a Shuffleboard object
    private RobotShuffleboard m_shuffleboard;

    //Declares variables for the motors that move the launcher flywheel, the feeder wheel, and the turret angle
    //Not all of these motors will be TalonFXs, those are placeholders until we know what kinds of motors we'll be using
    private TalonFX m_masterFlywheelMotor;
    private TalonFX m_slaveFlywheelMotor;
    private VictorSPX m_feederMotor;
    private TalonSRX m_turretMotor;

    //Declares variables for the encoders
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_turretEncoder;


    //declares our double solenoid to be used on our trajectory control system
    private DoubleSolenoid m_solenoid;

    //declares state enum to track our current trajectory control state
    private TrajectoryPosition m_state;

    private double m_feederCurrentSpeed;
    private double m_turretCurrentSpeed;
    private double m_flywheelCurrentSpeed;

    private double m_currentFlywheelVelocity = RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY;
    private double m_currentMaxTurretSpeed = RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED;
    private double m_proportionalConstant = RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT;

    //Boolean to store whether or not we are currently on target
    boolean m_onTarget = false;

    //variables for updating and storing the encoder ticks of the drivetrain and th turret every cycle
    double m_leftDriveEncoderTicks;
    double m_rightDriveEncoderTicks;
    double m_turretEncoderTicks;

    double m_onTargetLeftTicks = m_leftDriveEncoderTicks;
    double m_onTargetRightTicks = m_rightDriveEncoderTicks;
    double m_onTargetTurretTicks = m_turretEncoderTicks;   

    double m_angleToTarget = 0;
    
    //counts how many times we have cycled through targetAndLaunch while on target so we know when the ball has exited to robot
    double m_onTargetCounter = 0;

    double m_flywheelRevCounter = 0;

    boolean m_flywheelMotorReady = false;

    /**
     * Constructor for Launcher objects
     * @param limelightVision we pass in limelight to use in launch targeting
     */
    public Launcher(LimelightVision limelightVision, Drivetrain drivetrain, RobotShuffleboard shuffleboard){
        //Instantiates objects for the Launcher class
        m_limelightVision = limelightVision;
        m_drivetrain = drivetrain;
        m_shuffleboard = shuffleboard;

        //Instantiates the shuffleboard so the values for target flywheel speed on it can be used
        m_shuffleboard = shuffleboard;

        m_masterFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new VictorSPX(RobotMap.LauncherConstants.FEEDER_MOTOR_ID);
        m_turretMotor = new TalonSRX(RobotMap.LauncherConstants.TURRET_MOTOR_ID);

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
        m_feederCurrentSpeed = 0;
        m_feederMotor.set(ControlMode.PercentOutput, 0);
        m_turretCurrentSpeed = 0;
        m_turretMotor.set(ControlMode.PercentOutput, 0);
        m_flywheelCurrentSpeed = 0;
        m_masterFlywheelMotor.set(ControlMode.PercentOutput, 0);
        m_limelightVision.disableLEDs();

        m_currentMaxTurretSpeed = m_shuffleboard.getTurretValues();
        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();

        m_feederMotor.setNeutralMode(NeutralMode.Brake);
    }

    
    /**
     * Prepares launch sequence by turning turret towards the target and revving the launcher flywheel to the required speed
     */
    public void targetAndLaunch(double speed, double targetRpm){
        m_flywheelRevCounter++;

        setFlywheelSpeed(speed);
        System.out.println("Real Flywheel speed" + getRealSpeed());
        if(m_flywheelRevCounter >= 50){

            m_flywheelMotorReady = true;
        }
        else{
            m_flywheelMotorReady = false;
        }
        //Checks if our flywheel is at the target speed
        // if(getRealSpeed() > targetRpm){
        //     flywheelMotorReady = true;
        // }
        // else{
        //     flywheelMotorReady = false;
        // }

        m_onTarget = target();

        //Prints out a message telling the driver when our robot is ready to launch and moves game pieces into the flywheel
        if(m_onTarget && m_flywheelMotorReady){
            //setTrajectoryPosition(TrajectoryPosition.kUp);
            System.out.println("Commencing Launch Sequence");
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
            m_onTargetCounter++;
            if(m_onTargetCounter >= RobotMap.LauncherConstants.MAX_ON_TARGET_CYCLES){
                m_flywheelRevCounter = 0;
                m_onTarget = false;
            }
        }
        else{
            setFeederSpeed(0);
            m_onTargetCounter = 0;
        }
    }

    public void launch(){
        setFlywheelSpeed(m_currentFlywheelVelocity);
        target();
        System.out.println("Real Flywheel speed" + getRealSpeed());
        //Checks if our flywheel is at the target speed
        // if(getRealSpeed() > m_shuffleboard.getTargetFlywheelSpeed()){
        //     System.out.println("Commencing Launch Sequence");
        //     setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        // }
    }

    public boolean target(){
        boolean onTarget = false;

        m_leftDriveEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
        m_rightDriveEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        m_turretEncoderTicks = getTurretPosition();

        m_angleToTarget = m_limelightVision.xAngleToTarget();

        if (m_onTarget && !m_limelightVision.seeTarget()){
            if(m_leftDriveEncoderTicks != m_onTargetLeftTicks){
                onTarget = false;
            }
            else if(m_rightDriveEncoderTicks != m_onTargetRightTicks){
                onTarget = false;
            }
            else if(m_turretEncoderTicks != m_onTargetTurretTicks){
                m_onTarget = false;
            }
            else{
                onTarget = true;
                return onTarget;
            }
        }
        

        //this if statement makes it so if we don't see a target, don't run the method and instead print "No Target Detected"
        if(m_limelightVision.seeTarget()){
            System.out.println("Target Detected");

            //checks if the turret encoder is within the tolerated range, and if we're not print a message and adjust
            if(getTurretPosition() > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && getTurretPosition() < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //this if statements checks to see if we are within the tolerated error range, and if we are set turret bool to true
                if(m_angleToTarget < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && m_angleToTarget > -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    setTurretSpeed(0);
                    m_onTargetLeftTicks = m_leftDriveEncoderTicks;
                    m_onTargetRightTicks = m_rightDriveEncoderTicks;
                    m_onTargetTurretTicks = m_turretEncoderTicks;
                    onTarget = true;
                    System.out.print("Ready to Launch ------------------");
                }
                //if we are above the tolerated error range, turn the turret toward the tolerated error range
                else{
                    //Prints out a message telling the driver that our robot is not yet ready to launch and adjusts
                    System.out.println("Not Ready to Launch 1:" + m_angleToTarget);
                    onTarget = false;
                    setTurretSpeed(calcTurretSpeed(m_angleToTarget));
                }
            }
            //If we are to the left of our motor limit, print out a message and turn right
            else if(getTurretPosition() < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch 3");
                onTarget = false;
                setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }
            //If we are to the right of our motor limit, print out a message and turn left
            else if(getTurretPosition() > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                System.out.println("Not Ready to Launch 4");
                onTarget = false;
                setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }

        }
        else {
            System.out.println("No Target Detected");
        }

        return onTarget;
    }

    /**
     * Shoots any game pieces out at a low speed to eject game pieces we don't want
     */
    public void expel(){
        setTrajectoryPosition(TrajectoryPosition.kDown);
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
            setPiston(Value.kReverse);
        }
        else if (m_state == TrajectoryPosition.kDown){
            setPiston(Value.kForward);
        }
    }

    /**
     * @return current speed of flywheel motor in RPM
     */
    public double getRealSpeed(){
        return ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600) / 2048);
    }

    /**
     * Sets the speed of the launcher flywheel motor
     * This method is public for manual turret testing zeroing motors in CopilotController
     * @param speed desired speed 
     */
    public void setFlywheelSpeed(double speed){
        if(m_flywheelCurrentSpeed != speed){
            m_flywheelCurrentSpeed = speed;
            m_masterFlywheelMotor.set(ControlMode.PercentOutput, speed);
            m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
        }
    }

    /**
     * Sets the speed of the motor that moves the turret
     * This is method is public for manual turret testing and zeroing motors in CopilotController
     * @param speed desired speed (Positive for one direction, negative for the other)
     */
    public void setTurretSpeed(double speed){
        if(m_turretCurrentSpeed != speed){
            m_turretCurrentSpeed = speed;
            m_turretMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the feeder motor
     * @param speed desired speed
     */
    public void setFeederSpeed(double speed){
        if(m_feederCurrentSpeed != speed){
            m_feederCurrentSpeed = speed;
            m_feederMotor.set(ControlMode.PercentOutput, speed);
        }
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
    public double getTurretPosition(){
        return m_turretEncoder.getQuadraturePosition();
    }

    /**
     * Turns the turret back to center within a deadband of 10 ticks. If we are within the deadband, the turret stops moving and the encoder is set to zero
     */
    public void zeroTurretPosition(){
        if(getTurretPosition() >= RobotMap.LauncherConstants.TURRET_ENCODER_BAND){
            m_turretMotor.set(ControlMode.PercentOutput, RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        else if(getTurretPosition() <= -RobotMap.LauncherConstants.TURRET_ENCODER_BAND){
            m_turretMotor.set(ControlMode.PercentOutput, -RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        else {
            m_turretMotor.set(ControlMode.PercentOutput, 0);
            m_turretEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);

        }
    }

    /**
     * Calculates the speed to set the turret to based on how close we are to our taget. The closer we are, the slower we go.
     * @param currentAngle the current x angle to the target from the limelight
     * @return calculated speed of the turret
     */
    private double calcTurretSpeed(double currentAngle){
        //22 is the max angle our turret can turn to
        double t = RobotMap.LauncherConstants.MAX_TURRET_ROTATION - Math.abs(currentAngle);
        double output = m_currentMaxTurretSpeed * Math.exp(-1 * m_proportionalConstant * t); 
        if(output < RobotMap.LauncherConstants.MIN_TURRET_SPEED){
            output = RobotMap.LauncherConstants.MIN_TURRET_SPEED;
        }

        if (currentAngle < 0){
            output *= -1;
        }
        return output;
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
        if((getRealSpeed() / RobotMap.LauncherConstants.MAX_FLYWHEEL_RPM) >= 0.7){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 20 ft away into the low hub
     * The getRealSPeed is being converted to a percent speed in the if statement. The Max RPM value it can go is 6380
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void lowPreset20(){
        setTrajectoryPosition(TrajectoryPosition.kDown);
        setFlywheelSpeed(0.9);
        if((getRealSpeed() / RobotMap.LauncherConstants.MAX_FLYWHEEL_RPM) >= 0.9){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 10 ft away into the high hub
     * The getRealSPeed is being converted to a percent speed in the if statement. The Max RPM value it can go is 6380
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void highPreset10(){
        setTrajectoryPosition(TrajectoryPosition.kUp);
        setFlywheelSpeed(0.7);
        if((getRealSpeed() / RobotMap.LauncherConstants.MAX_FLYWHEEL_RPM) >= 0.7){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }

    /**
     * This is the preset for launching the ball 20 ft away into the high hub
     * The getRealSPeed is being converted to a percent speed in the if statement. The Max RPM value it can go is 6380
     * TODO: Test flywheel velocity needed to reach our target
     */
    public void highPreset20(){
        setTrajectoryPosition(TrajectoryPosition.kUp);
        setFlywheelSpeed(0.9);
        if((getRealSpeed() / RobotMap.LauncherConstants.MAX_FLYWHEEL_RPM) >= 0.9){
            setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
    }
    
    public void zeroFlywheelRevCounter(){
        m_flywheelRevCounter = 0;
    }
}