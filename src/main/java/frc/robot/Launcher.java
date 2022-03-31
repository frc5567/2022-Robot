package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Intake.IntakeState;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


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

    //Enum that stores the current state of our flywheel during launch. Preparing: The flywheel is not yet up to speed. Recovery: The flwyheel is returning to speed. Ready: The flywheel is ready to launch
    private enum LauncherState{
        kPreparing,

        kRecovery,

        kReady;
    }

    //Declares limelight object
    private LimelightVision m_limelightVision;
    private Drivetrain m_drivetrain;
    private Intake m_intake;
    private RobotShuffleboard m_shuffleboard;

    //Declares variables for the motors that move the launcher flywheel, the feeder wheel, and the turret angle
    //Not all of these motors will be TalonFXs, those are placeholders until we know what kinds of motors we'll be using
    private WPI_TalonFX m_masterFlywheelMotor;
    private WPI_TalonFX m_slaveFlywheelMotor;
    private VictorSPX m_feederMotor;
    public TalonSRX m_turretMotor;

    //Declares variables for the encoders
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_turretEncoder;


    //declares our double solenoid to be used on our trajectory control system
    // private DoubleSolenoid m_solenoid;

    //declares state enum to track our current trajectory control state
    private TrajectoryPosition m_state;

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
    double m_feedingCounter = 0;

    double m_flywheelRevCounter = 0;

    boolean m_flywheelMotorReady = false;

    boolean m_secondBall = false;

    TalonFXConfiguration m_masterConfig;
    TalonFXConfiguration m_slaveConfig;

    int m_launcherAtSpeedCount = 0;

    private double m_feederCurrentSpeed;
    private double m_turretCurrentSpeed;
    private double m_flywheelCurrentSpeed;

    private double m_currentTargetFlywheelRpm = RobotMap.LauncherConstants.TARGET_FLYWHEEL_RPM;
    private double m_currentMaxTurretSpeed = RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED;
    private double m_proportionalConstant = RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT;

    double m_currentKP;
    double m_currentKI;
    double m_currentKD;
    double m_currentKF;
    
    public boolean m_atRPM;

    public double m_dist;

    /**
     * Constructor for Launcher objects
     * @param limelightVision we pass in limelight to use in launch targeting
     * @param drivetrain we pass in drivetrain to get encoder ticks/position
     * @param shuffleboard we pass in shuffleboard for getTurretValues and getFlywheelVelocity
     * @param intake we pass in intake to extend the intake in targetAndLaunch
     */
    public Launcher(LimelightVision limelightVision, Drivetrain drivetrain, Intake intake){
        //Instantiates objects for the Launcher class
        m_limelightVision = limelightVision;
        m_drivetrain = drivetrain;
        m_intake = intake;

        m_masterFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.MASTER_FLYWHEEL_FALCON_ID);
        m_slaveFlywheelMotor = new WPI_TalonFX(RobotMap.LauncherConstants.SLAVE_FLYWHEEL_FALCON_ID);
        m_feederMotor = new VictorSPX(RobotMap.LauncherConstants.FEEDER_MOTOR_ID);
        m_turretMotor = new TalonSRX(RobotMap.LauncherConstants.TURRET_MOTOR_ID);

        m_flywheelEncoder = new SensorCollection (m_masterFlywheelMotor);
        m_turretEncoder = new SensorCollection (m_turretMotor);

        // m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.LauncherConstants.DOUBLESOLENOID_ANGLE_DOWN_PORT, RobotMap.LauncherConstants.DOUBLESOLENOID_ANGLE_UP_PORT);
    
        m_state = TrajectoryPosition.kUnkown;

        m_masterConfig = new TalonFXConfiguration();
	    m_slaveConfig = new TalonFXConfiguration();
    }

    /**
     * Initialization method for Launcher. Currently only zeros encoders and sets our default trajectory position to up
     */
    public void init(){
        // setTrajectoryPosition(TrajectoryPosition.kDown);
        m_feederMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveFlywheelMotor.setInverted(true);
        m_feederCurrentSpeed = 0;
        m_feederMotor.set(ControlMode.PercentOutput, 0);
        m_turretCurrentSpeed = 0;
        m_turretMotor.set(ControlMode.PercentOutput, 0);
        m_flywheelCurrentSpeed = 0;
        //m_masterFlywheelMotor.set(ControlMode.PercentOutput, 0);
        m_limelightVision.disableLEDs();

        m_feederMotor.setNeutralMode(NeutralMode.Brake);
        //m_turretMotor.setNeutralMode(NeutralMode.Brake);
        m_atRPM = false;

        configTalonPID();

        // m_masterFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        // m_masterFlywheelMotor.configSelectedFeedbackSensor(m_masterFlywheelMotor.getSensorCollection()., 0, 30);
        // m_slaveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // m_masterConfig.remoteFilter0.remoteSensorDeviceID = m_slaveFlywheelMotor.getDeviceID(); 
		// m_masterConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;
        // m_masterConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

        // m_masterConfig.slot2.kP = m_currentKP;
        // m_masterConfig.slot2.kI = m_currentKI;
        // m_masterConfig.slot2.kD = m_currentKD;
        // m_masterConfig.slot2.kF = m_currentKF;
        // m_masterConfig.slot2.integralZone = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kIzone;
		// m_masterConfig.slot2.closedLoopPeakOutput = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kPeakOutput;

        // m_masterConfig.auxPIDPolarity = false;

        // m_masterConfig.neutralDeadband = RobotMap.LauncherConstants.NEUTRAL_DEADBAND;
        // m_slaveConfig.neutralDeadband = RobotMap.LauncherConstants.NEUTRAL_DEADBAND;

        // int closedLoopTimeMs = 1;

        // m_masterConfig.slot0.closedLoopPeriod = closedLoopTimeMs;
		// m_masterConfig.slot1.closedLoopPeriod = closedLoopTimeMs;
		// m_masterConfig.slot2.closedLoopPeriod = closedLoopTimeMs;
		// m_masterConfig.slot3.closedLoopPeriod = closedLoopTimeMs;

        // m_masterConfig.motionAcceleration = 2000; //(distance units per 100 ms) per second
		// m_masterConfig.motionCruiseVelocity = 2000; //distance units per 100 ms

        // m_masterFlywheelMotor.configAllSettings(m_masterConfig);
        // m_slaveFlywheelMotor.configAllSettings(m_slaveConfig);

        // m_masterFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, RobotMap.TIMEOUT_MS);
		// m_masterFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, RobotMap.TIMEOUT_MS);
		// m_slaveFlywheelMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, RobotMap.TIMEOUT_MS);

        zeroEncoders();

        // m_masterFlywheelMotor.selectProfileSlot(RobotMap.LauncherConstants.PID_SLOT, RobotMap.LauncherConstants.PID_MODE);
    }

    public void periodic(){
        setGains();
    }

    public boolean runPID(double desiredRpm){
        boolean atSpeed = false;

        double target_unitsPer100ms = desiredRpm * (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION / 600.0;	//RPM -> Native units

        /* Configured for Velocity Closed Loop on Integrated Sensors' Sum and Arbitrary FeedForward */
        m_masterFlywheelMotor.set(TalonFXControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, 0.0);
		m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);

        double actualRpm = ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600.0) / 2048.0);//(m_masterFlywheelMotor.getSelectedSensorVelocity() / (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION * 600f);
        
        if((actualRpm >= desiredRpm - RobotMap.LauncherConstants.FLYWHEEL_RPM_BOUND) && (actualRpm <= desiredRpm + RobotMap.LauncherConstants.FLYWHEEL_RPM_BOUND)){
            atSpeed = true;
            m_atRPM = true;
        }
        else{
            m_atRPM = false;
        }
        
        return atSpeed;
    }

    private void launchPID(){
        m_masterFlywheelMotor.selectProfileSlot(RobotMap.LauncherConstants.PID_SLOT, RobotMap.LauncherConstants.PID_MODE);
        //System.out.println("launching "+ ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600.0) / 2048.0)+ " \tOutput: "+ m_masterFlywheelMotor.getMotorOutputPercent() + " Distance: " + m_limelightVision.distToTarget());
        m_dist = m_limelightVision.distToTarget();
        System.out.println("Distance: " + m_dist);
        //double desiredRpm = (3119.8 + (19.002 * m_dist) - (.1949171* Math.pow(m_dist, 2)) + (.000954319 * Math.pow(m_dist, 3)));
        double desiredRpm = (0.0047 * Math.pow(m_dist, 3) - 1.2642 * Math.pow(m_dist, 2) + 121.27 * (m_dist) - 127.68);
       

        // runPID(2000);
        //runPID(desiredRpm)
        if(runPID(desiredRpm)){
            m_launcherAtSpeedCount++;

            double ticks = m_masterFlywheelMotor.getSelectedSensorVelocity();
            double dticks = desiredRpm * (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION / 600.0;
            System.out.println("Desired RPM:    [" + desiredRpm + "]   Actual RPM:    [" + ((ticks * 600.0) / 2048.0) + "]");
            System.out.println("Desired ticks:  [" + dticks +     "]   Actual ticks:  [" + ticks + "]");
        }
        else{
            m_launcherAtSpeedCount = 0;
        }
    }

    public void targetAndLaunch(){
        // m_masterFlywheelMotor.selectProfileSlot(RobotMap.LauncherConstants.PID_SLOT, RobotMap.LauncherConstants.PID_MODE);
        // m_masterFlywheelMotor.set(TalonFXControlMode.Velocity, 2000, DemandType.ArbitraryFeedForward, 0.1);
        // System.out.println("launching "+m_masterFlywheelMotor.getSelectedSensorVelocity());
        // target();
        launchPID();

        boolean onTarget = target();
        
        if(m_launcherAtSpeedCount > 10 && onTarget){
            System.out.println("launching (count:" + m_launcherAtSpeedCount + ")");
            feedLauncher();
        }

        if(m_secondBall){
            m_intake.indexing();
        }
    }

    private void feedLauncher(){
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        m_feedingCounter++;
        if(m_feedingCounter > RobotMap.LauncherConstants.MAX_FEEDING_CYCLES){
            setFeederSpeed(0);
            m_feedingCounter = 0;
            m_launcherAtSpeedCount = 0;
            System.out.println("completed feed");
            if(m_secondBall){
                m_secondBall = false;
            }
            else{
                m_secondBall = true;
            }

        }
    }



    
    /**
     * Prepares launch sequence by turning turret towards the target and revving the launcher flywheel to the required speed
     */
    // public void targetAndLaunch(double speed){

    //     //Sets the intake to extended so the game pieces don't get jammed
    //     m_intake.setIntakeExtension(IntakeState.kExtended);
    //     m_flywheelRevCounter++;

    //     setFlywheelSpeed(speed);
    //     //System.out.println("Real Flywheel speed" + getRealSpeed());
    //     if(m_flywheelRevCounter >= 50){

    //         m_flywheelMotorReady = true;
    //     }
    //     else{
    //         m_flywheelMotorReady = false;
    //     }
    //     //Checks if our flywheel is at the target speed
    //     // if(getRealSpeed() > targetRpm){
    //     //     flywheelMotorReady = true;
    //     // }
    //     // else{
    //     //     flywheelMotorReady = false;
    //     // }

    //     m_onTarget = target();

    //     //Prints out a message telling the driver when our robot is ready to launch and moves game pieces into the flywheel
    //     if(m_onTarget && m_flywheelMotorReady){
    //         //setTrajectoryPosition(TrajectoryPosition.kUp);
    //         System.out.println("Commencing Launch Sequence");
    //         setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    //         m_onTargetCounter++;
    //         if(m_onTargetCounter >= RobotMap.LauncherConstants.MAX_ON_TARGET_CYCLES){
    //             m_flywheelRevCounter = 0;
    //             m_onTarget = false;
    //         }
    //     }
    //     else{
    //         setFeederSpeed(0);
    //         m_onTargetCounter = 0;
    //     }
    // }

    public void launch(){
        //setFlywheelSpeed(m_currentTargetFlywheelVelocity);
        //System.out.println("Real Flywheel speed" + getRealSpeed() + "\tOutput: "+m_masterFlywheelMotor.getMotorOutputPercent());
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
            //System.out.println("Target Detected");

            //checks if the turret encoder is within the tolerated range, and if we're not print a message and adjust
            if(getTurretPosition() > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && getTurretPosition() < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //this if statements checks to see if we are within the tolerated error range, and if we are set turret bool to true
                if(m_angleToTarget < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR_RIGHT && m_angleToTarget > RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR_LEFT){
                    setTurretSpeed(0);
                    m_onTargetLeftTicks = m_leftDriveEncoderTicks;
                    m_onTargetRightTicks = m_rightDriveEncoderTicks;
                    m_onTargetTurretTicks = m_turretEncoderTicks;
                    onTarget = true;
                    //System.out.print("Ready to Launch ------------------");
                }
                //if we are above the tolerated error range, turn the turret toward the tolerated error range
                else{
                    //Prints out a message telling the driver that our robot is not yet ready to launch and adjusts
                    //System.out.println("Not Ready to Launch 1:" + m_angleToTarget);
                    onTarget = false;
                    setTurretSpeed(turretSpeed(m_angleToTarget));
                }
            }
            //If we are to the left of our motor limit, print out a message and turn right
            else if(getTurretPosition() < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //System.out.println("Not Ready to Launch 3");
                onTarget = false;
                setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }
            //If we are to the right of our motor limit, print out a message and turn left
            else if(getTurretPosition() > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
                //System.out.println("Not Ready to Launch 4");
                onTarget = false;
                setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
            }

        }
        else {
            //System.out.println("No Target Detected");
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
        // System.out.println("RPM: " + m_masterFlywheelMotor.getSelectedSensorVelocity());
        // System.out.println("RPM master0: "+ m_masterFlywheelMotor.getSelectedSensorVelocity() + "\tPct Output: " + m_masterFlywheelMotor.getMotorOutputPercent());
        return ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600) / 2048);
    }

    /**
     * Sets the speed of the launcher flywheel motor
     * This method is public for manual turret testing zeroing motors in CopilotController
     * @param speed desired speed 
     */
    public void setFlywheelSpeed(double speed){
        m_masterFlywheelMotor.set(speed);
        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
        // if(m_flywheelCurrentSpeed != speed){
        //     m_flywheelCurrentSpeed = speed;
        // }
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
        // m_solenoid.set(value);
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
            m_turretMotor.set(ControlMode.PercentOutput, turretSpeed(getTurretPosition()));
        }
        else if(getTurretPosition() <= -RobotMap.LauncherConstants.TURRET_ENCODER_BAND){
            m_turretMotor.set(ControlMode.PercentOutput, turretSpeed(getTurretPosition()));
        }
        else {
            m_turretMotor.set(ControlMode.PercentOutput, 0);
            m_turretEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);

        }
    }

    /**
     * Calculates the speed to set the turret to based on how close we are to our taget. The closer we are, the slower we go.
     * Do not use for centering the turret because for some reason it doesn't work
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

    private double turretSpeed(double angleOffset){
        double turretSpeed;
        double sign = Math.signum(angleOffset);
        //System.out.println(angleOffset);
        //System.out.println("hi\n\nhi\n\nhi***********************************************\n**********************************************");
        if(Math.abs(angleOffset) > 15){
            turretSpeed = sign*m_currentMaxTurretSpeed/10;
        }
        else if (Math.abs(angleOffset) > 10){
            turretSpeed = sign*m_currentMaxTurretSpeed/30;
        }
        else if (Math.abs(angleOffset) > 6.5){
            turretSpeed = sign*m_currentMaxTurretSpeed/40;
        }
        else{
            turretSpeed = sign*RobotMap.LauncherConstants.MIN_TURRET_SPEED;
        }
        return turretSpeed;
    }
    
    public void zeroFlywheelRevCounter(){
        m_flywheelRevCounter = 0;
    }

    public double getCurrentPercent(){
        return m_masterFlywheelMotor.getMotorOutputPercent();
    }

    public void setMaxTurretSpeed(double speed){
        m_currentMaxTurretSpeed = speed;
    }

    public void setProportionalConstant(double constant){
        m_proportionalConstant = constant;
    }

    public void setTargetFlywheelRpm(double speed){
        m_currentTargetFlywheelRpm = speed;
    }

    public void setKP(double P){
        m_currentKP = P;
        // m_masterConfig.slot2.kP = m_currentKP;
        // m_masterFlywheelMotor.configAllSettings(m_masterConfig);
    }

    public void setKI(double I){
        m_currentKI = I;
    }

    public void setKD(double D){
        m_currentKD = D;
    }

    public void setKF(double F){
        m_currentKF = F;
    }

    public void resetSecondBallTracker(){
        m_secondBall = false;
    }

    private void setGains(){
        if(m_masterConfig.slot2.kP != m_currentKP){
            m_masterConfig.slot2.kP = m_currentKP; //0.57
        }
        
        if(m_masterConfig.slot2.kI != m_currentKI){
            m_masterConfig.slot2.kI = m_currentKI; //0
        }
        
        if(m_masterConfig.slot2.kD != m_currentKD){
            m_masterConfig.slot2.kD = m_currentKD; //16
        }

        if(m_masterConfig.slot2.kF != m_currentKF){
            m_masterConfig.slot2.kF = m_currentKF; //0.05
        }
        
    }

    private void configTalonPID(){
        m_masterFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_masterConfig.slot2.kP = m_currentKP; //0.57
        m_masterConfig.slot2.kI = m_currentKI; //0
        m_masterConfig.slot2.kD = m_currentKD; //16
        m_masterConfig.slot2.kF = m_currentKF; //0.05
        m_masterConfig.slot2.integralZone = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kIzone;
		m_masterConfig.slot2.closedLoopPeakOutput = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kPeakOutput;
        m_masterConfig.slot2.closedLoopPeriod = 1;
        m_masterFlywheelMotor.configPeakOutputForward(1);
        m_masterFlywheelMotor.configPeakOutputReverse(0);
        m_masterFlywheelMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

        m_slaveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        m_masterConfig.remoteFilter0.remoteSensorDeviceID = m_slaveFlywheelMotor.getDeviceID();
        m_masterConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonFX_SelectedSensor;

        m_masterFlywheelMotor.configAllSettings(m_masterConfig);
        m_slaveFlywheelMotor.configAllSettings(m_slaveConfig);

        m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);
    }
}