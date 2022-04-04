package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


public class Launcher{
    //Declares limelight object for targeting
    private LimelightVision m_limelightVision;
    //Declares drivetrain object to check encoder ticks
    private Drivetrain m_drivetrain;
    //Declares intake to manipulate magazine wheels while automatically launching
    private Intake m_intake;

    //Declares variables for the motors that move the launcher flywheel motors, the feeder wheel, and the turret angle
    private WPI_TalonFX m_masterFlywheelMotor;
    private WPI_TalonFX m_slaveFlywheelMotor;
    private VictorSPX m_feederMotor;
    public TalonSRX m_turretMotor;

    //Declares the encoders to calculate rpm and track the current position of the turret
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_turretEncoder;

    //Boolean to store whether or not we are currently on target
    boolean m_onTarget = false;

    //variables for updating and storing the encoder ticks of the drivetrain and the turret every cycle
    double m_leftDriveEncoderTicks;
    double m_rightDriveEncoderTicks;
    double m_turretEncoderTicks;

    //Stores the ticks of the drivetrain/turret when we are on target to check if we have moved since we were on target
    double m_onTargetLeftTicks = m_leftDriveEncoderTicks;
    double m_onTargetRightTicks = m_rightDriveEncoderTicks;
    double m_onTargetTurretTicks = m_turretEncoderTicks;   

    //Tracks the cycles the flywheel has been at speed so we know if we overshoot
    int m_launcherAtSpeedCount = 0;

    //Member variables used to make sure we aren't calling on the CAN bus constantly by repetedly setting the motors to zero 
    private double m_feederCurrentSpeed;
    private double m_turretCurrentSpeed;

    //Stores target flywheel RPM value from the shuffleboard for testing 
    //Commented out because it is currently unused, but should be used if we want to do further distance testing
    //private double m_currentTargetFlywheelRpm = RobotMap.LauncherConstants.DEFAULT_TARGET_FLYWHEEL_RPM;
    
    //Stores max turret speed value from the shuffleboard to change how fast the turret moves
    private double m_currentMaxTurretSpeed = RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED;
    //Stores target flywheel percent power from the shuffleboard for manual launching
    private double m_currentFlywheelPercentPower = RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_PERCENT_POWER;

    //PID configuration objects for the master and slave flywheel motors
    TalonFXConfiguration m_masterConfig;
    TalonFXConfiguration m_slaveConfig;

    //Variables to store our gains from the shuffleboard
    double m_currentKP;
    double m_currentKI;
    double m_currentKD;
    double m_currentKF;
    
    //Variable for storing whether or not we're at our target RPM for the boolean box on the shuffleboard
    public boolean m_atRPM;

    int m_feedingCounter = 0;

    /**
     * Constructor for Launcher objects
     * @param limelightVision we pass in limelight to use in launch targeting
     * @param drivetrain we pass in drivetrain to get encoder ticks/position
     * @param shuffleboard we pass in shuffleboard for PID tuning and quickly editing launcher constants
     * @param intake we pass in intake to minipulate intake extension and magazine motors
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

        //Assigns encoders to their respective motors
        m_flywheelEncoder = new SensorCollection (m_masterFlywheelMotor);
        m_turretEncoder = new SensorCollection (m_turretMotor);

        //Instantiates configuration objects for the flywheel motors
        m_masterConfig = new TalonFXConfiguration();
	    m_slaveConfig = new TalonFXConfiguration();
    }

    /**
     * This method is called once as soon as the robot is enabled. 
     * Configures all launcher motors and flywheel PID
     */
    public void init(){
        m_feederMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveFlywheelMotor.setInverted(true);
        m_feederCurrentSpeed = 0;
        m_feederMotor.set(ControlMode.PercentOutput, 0);
        m_turretCurrentSpeed = 0;
        m_turretMotor.set(ControlMode.PercentOutput, 0);
        m_limelightVision.disableLEDs();

        m_feederMotor.setNeutralMode(NeutralMode.Brake);
        m_turretMotor.setNeutralMode(NeutralMode.Brake);
        m_atRPM = false;

        configTalonPID();

        zeroEncoders();
    }

    /**
     * Called many times a second while the robot is enabled
     */
    public void periodic(){
        //Updates gains in the code to the gains in the shuffleboard
        setGains();
    }

    /**
     * Applies PID to our flywheel motors and checks if we have revved up to our desired RPM
     * @param desiredRpm the RPM we want the flywheel motors at
     * @return True if we are at our desired rpm, false if we are not
     */
    public boolean runPID(double desiredRpm){
        boolean atSpeed = false;

        double target_unitsPer100ms = desiredRpm * (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION / 600.0;	//RPM -> Native units

        /* Configured for Velocity Closed Loop on Integrated Sensors' Sum and Arbitrary FeedForward */
        m_masterFlywheelMotor.set(TalonFXControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, 0.0);
		m_slaveFlywheelMotor.follow(m_masterFlywheelMotor);

        //Calculates our current actual RPM from raw encoder ticks per 100ms
        double actualRpm = ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600.0) / 2048.0);//(m_masterFlywheelMotor.getSelectedSensorVelocity() / (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION * 600f);
        
        //If we are within our RPM bounds, return true
        if((actualRpm >= desiredRpm - RobotMap.LauncherConstants.FLYWHEEL_RPM_BOUND) && (actualRpm <= desiredRpm + RobotMap.LauncherConstants.FLYWHEEL_RPM_BOUND)){
            atSpeed = true;
            m_atRPM = true;
        }
        else{
            m_atRPM = false;
        }
        
        return atSpeed;
    }

    /**
     * Calculates our desired RPM based on distance and runs our PID
     */
    private void launchPID(){
        //tracks the current distance from the limelight to the hub in inches
        double dist = m_limelightVision.distToTarget();
        //Applies PID configurations to the master flywheel motor
        m_masterFlywheelMotor.selectProfileSlot(RobotMap.LauncherConstants.PID_SLOT, RobotMap.LauncherConstants.PID_MODE);
        //Prints out our current distance
        //Calculates desired RPM based on our distance from the hub
        double desiredRpm = (0.0078 * Math.pow(dist, 3) - 2.3105 * Math.pow(dist, 2) + 236.44 * (dist) - 4384.4);
        // System.out.println("Distance: " + dist + "\tDesRPM: "+ desiredRpm);
        
        if(runPID(desiredRpm)){
            //Adds to our at speed count every cycle we're at speed
            m_launcherAtSpeedCount++;
            // System.out.println("Launcher at speed count: " + m_launcherAtSpeedCount );

            //Sysouts for testing
            //double ticks = m_masterFlywheelMotor.getSelectedSensorVelocity();
            //double dticks = desiredRpm * (double)RobotMap.LauncherConstants.TICKS_PER_ROTATION / 600.0;
            // System.out.println("Desired RPM:    [" + desiredRpm + "]   Actual RPM:    [" + ((ticks * 600.0) / 2048.0) + "]");
            // System.out.println("Desired ticks:  [" + dticks +     "]   Actual ticks:  [" + ticks + "]");
        }
    }

    /**
     * Method that is run when the target and launch button on the gamepad is pressed. 
     * Spins up the flywheel and targets, and when the flywheel is up to speed and the turret is on target activate the feeder
     */
    public void targetAndLaunch(){
        m_intake.indexing();
        launchPID();

        boolean onTarget = target();
        
        if(m_launcherAtSpeedCount > 10 && onTarget){
            // System.out.println("launching (count:" + m_launcherAtSpeedCount + ")");
            feedLauncher();
        }
    }

    /**
     * Feeds the ball in the second position into the flywheel
     */
    private void feedLauncher(){
        // System.out.println("FEEDING -----------------------------------");
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        //Tracks how many loops we activate the feeder wheel
        m_feedingCounter++;
        if(m_feedingCounter > RobotMap.LauncherConstants.MAX_FEEDING_CYCLES){
            setFeederSpeed(0);
            //Once we feed the ball into the launcher, reset the launch cycle
            m_feedingCounter = 0;
            m_launcherAtSpeedCount = 0;
        }
    }
    
    /**
     * Powers the flywheel motor to the speed set on the shuffleboard for manual launching
     */
    public void launch(){
        setFlywheelSpeed(m_currentFlywheelPercentPower);
    }

    /**
     * Uses the limelight to find a target and turn the turret toward it
     * @return True if our turret is on target, False if the turret is not on target
     */
    public boolean target(){
        //Updates the angle from the center of limelight every loop
        double angleToTarget = m_limelightVision.xAngleToTarget();
        //We are never on target by default
        boolean onTarget = false;

        //Updates encoder ticks every loop to check if we have moved 
        m_leftDriveEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
        m_rightDriveEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        m_turretEncoderTicks = getTurretPosition();

        System.out.println("Angle to Target: " + angleToTarget);

        //Checks if we have moved since we were on target last; if we have, record that we are no longer on target.
        //This is to make sure if we lose vision of the target we can record that we are on target until either the robot or the turret moves
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

        //Checks if we can see the target
        if(m_limelightVision.seeTarget()){
            //Checks if the turret is within the bounds we set in the code
            if(turretInBounds()){
                if(angleToTarget < RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR && angleToTarget > -RobotMap.LauncherConstants.TOLERATED_TURRET_ERROR){
                    setTurretSpeed(0);
                    m_onTargetLeftTicks = m_leftDriveEncoderTicks;
                    m_onTargetRightTicks = m_rightDriveEncoderTicks;
                    m_onTargetTurretTicks = m_turretEncoderTicks;
                    onTarget = true;
                    // System.out.print("ON TARGET");
                }
                //if we are outside of the tolerated error range, turn the turret toward the tolerated error range
                else{
                    //Prints out a message telling the driver that our robot is not yet ready to launch and adjusts
                    //System.out.println("Not Ready to Launch 1:" + m_angleToTarget);
                    onTarget = false;
                    setTurretSpeed(calcTurretSpeedToAngle(angleToTarget));
                }
            }
            //If the turret is outside of its bounds, return false because we cannot be on target if we are outside of the bands
            else{
                onTarget = false;
            }
        }
        return onTarget;
    }

    /**
     * Shoots any game pieces out at a low speed to eject game pieces we don't want
     */
    public void expel(){
        setFlywheelSpeed(RobotMap.LauncherConstants.EXPEL_SPEED);
        setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }

    /**
     * @return current speed of flywheel motor in RPM
     */
    public double getRealSpeed(){
        return ((m_masterFlywheelMotor.getSelectedSensorVelocity() * 600) / 2048);
    }

    /**
     * Sets the speed of the launcher flywheel motors
     * @param speed desired speed of the master and slave flywheel motors
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
        //Only sets the speed of the motor once each time we want it to change speed to limit CAN usage
        if(m_turretCurrentSpeed != speed){
            m_turretCurrentSpeed = speed;
            m_turretMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the feeder motor
     * @param speed desired speed of the feeder motor
     */
    public void setFeederSpeed(double speed){
        //Only sets the speed of the motor once each time we want it to change speed to limit CAN usage
        if(m_feederCurrentSpeed != speed){
            m_feederCurrentSpeed = speed;
            m_feederMotor.set(ControlMode.PercentOutput, speed);
        }
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
     * Turns the turret back to center within a deadband of 50 ticks on each side. If we are within the deadband, the turret stops moving and the encoder is set to zero
     */
    public void zeroTurretPosition(){
        double turretPosition = getTurretPosition();
        //Checks if the turret is within the outside bounds 
        if(turretInBounds()){
            if(turretPosition >= RobotMap.LauncherConstants.TURRET_ENCODER_BAND){
                //System.out.println("MOVE RIGHT");
                m_turretMotor.set(ControlMode.PercentOutput, calcTurretSpeedToTicks(turretPosition));
            }
            else if(turretPosition <= -RobotMap.LauncherConstants.TURRET_ENCODER_BAND){
                //System.out.println("MOVE LEFT");
                m_turretMotor.set(ControlMode.PercentOutput, calcTurretSpeedToTicks(turretPosition));
            }
            else {
                //System.out.println("CENTERED");
                m_turretMotor.set(ControlMode.PercentOutput, 0);
                m_turretEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);
            }
        }
    }


    /**
     * Method to check whether or not we are inside our outer limits on the turret. If we aren't, turn the turret until we are.
     * @return True if we are inside our bounds, False if we are outside our bounds
     */
    private boolean turretInBounds(){
        boolean inBounds = false;
        //Updates the current position of the turret in encoder ticks every loop
        double turretPosition = getTurretPosition();
        //checks if the turret encoder is within the tolerated range, and if we're not print a message and adjust
        if(turretPosition > -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT && turretPosition < RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
            setTurretSpeed(0);
            inBounds = true;
        }
        //If we are to the left of our motor limit, print out a message and turn right
        else if(turretPosition < -RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
            inBounds = false;
            setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        //If we are to the right of our motor limit, print out a message and turn left
        else if(turretPosition > RobotMap.LauncherConstants.TURRET_ENCODER_LIMIT){
            inBounds = false;
            setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        return inBounds;
    }

    /**
     * Calculates the necessary speed for turning the turret to a specific ANGLE 
     * @param angleOffset the difference between the turret's current angle and the desired angle. Takes negatives into account
     * @return the speed of the turret based on our current offset
     */
    private double calcTurretSpeedToAngle(double angleOffset){
        double turretSpeed;
        //Records the sign (Positive or negative) of the angleoffset
        double sign = Math.signum(angleOffset);
        // System.out.println("angle offset: " + angleOffset);
        // System.out.println("hi\n\nhi\n\nhi***********************************************\n**********************************************");
        // Danny's funny sysout ^

        //If we are of by greater than 12 degrees, set the speed of the target to the max speed set on the shuffleboard devided by 1.5
        if(Math.abs(angleOffset) > RobotMap.LauncherConstants.TURRET_SPEED_ANGLE_3){
            turretSpeed = sign*m_currentMaxTurretSpeed/1.5;     
        }
        //If we are of by greater than 8 degrees, set the speed of the target to the max speed set on the shuffleboard devided by 2
        else if (Math.abs(angleOffset) > RobotMap.LauncherConstants.TURRET_SPEED_ANGLE_2){
            turretSpeed = sign*m_currentMaxTurretSpeed/2;
        }
        //If we are of by greater than 3 degrees, set the speed of the target to 0.3
        else if (Math.abs(angleOffset) > RobotMap.LauncherConstants.TURRET_SPEED_ANGLE_1){
            turretSpeed = sign*RobotMap.LauncherConstants.LOW_TURRET_SPEED;
        }
        //If we are of by less than 3 degrees, set the speed of the target to 0.1
        else{
            turretSpeed = sign*RobotMap.LauncherConstants.MIN_TURRET_SPEED;
        }
        //System.out.println("Turret Speed: " + turretSpeed);
        return turretSpeed;
    }

    /**
     * Calculates the necessary speed for turning the turret to a specific location calculated with TICKS
     * @param tickOffset the difference between the turret's current ticks and the desired ticks. Takes negatives into account
     * @return the speed of the turret based on our current offset
     */
    private double calcTurretSpeedToTicks(double tickOffset){
        double turretSpeed = 0;
        //Records the sign (Positive or negative) of the tickOffset
        double sign = Math.signum(tickOffset);

        //If we are of by greater than 3000 ticks, set the speed of the target to the max speed set on the shuffleboard
        if(Math.abs(tickOffset) > RobotMap.LauncherConstants.TURRET_SPEED_TICKS_2){
            turretSpeed = sign*m_currentMaxTurretSpeed;
        }
        //If we are of by greater than 2000 ticks, set the speed of the target to the max speed set on the shuffleboard devided by 4
        else if (Math.abs(tickOffset) > RobotMap.LauncherConstants.TURRET_SPEED_TICKS_1){
            turretSpeed = sign*m_currentMaxTurretSpeed/4;
        }
        //If we are of by less than 2000 ticks, set the speed of the target to 0.1
        else{
            turretSpeed = sign*RobotMap.LauncherConstants.MIN_TURRET_SPEED;
        }
        //System.out.println("Turret Speed: " + turretSpeed);
        return turretSpeed;
    }

    /**
     * @return the curret percent power being sent to the flywheel motor
     */
    public double getCurrentPercent(){
        return m_masterFlywheelMotor.getMotorOutputPercent();
    }

    /**
     * Sets the maximum percent power of the turret 
     * @param speed desired maximum percent power of the turret motor
     */
    public void setMaxTurretSpeed(double speed){
        m_currentMaxTurretSpeed = speed;
    }

    /**
     * Sets the percent power of the flywheel motor for use in manual launching
     * @param speed desired percent power of the flywheel motor
     */
    public void setFlywheelPercentPower(double speed){
        m_currentFlywheelPercentPower = speed;
    }

    /**
     * Sets the target flywheel RPM for use in the flywheel PID
     * Commented out because it is currently unused, but should be used if we want to do further distance testing
     * @param speed desired target flywheel RPM
     */
    // public void setTargetFlywheelRpm(double speed){
    //     m_currentTargetFlywheelRpm = speed;
    // }

    /**
     * Sets the Proportional gain in our flywheel PID
     * @param P desired P gain
     */
    public void setKP(double P){
        m_currentKP = P;
    }

    /**
     * Sets the Integral gain in our flywheel PID
     * @param I desired I gain
     */
    public void setKI(double I){
        m_currentKI = I;
    }

    /**
     * Sets the Derivative gain in our flywheel PID
     * @param D desired D gain
     */
    public void setKD(double D){
        m_currentKD = D;
    }

    /**
     * Sets the Feed Forward gain in our flywheel PID
     * @param F desired F gain
     */
    public void setKF(double F){
        m_currentKF = F;
    }

    /**
     * Sets the turret and feeder motors to coast mode. For use in disabled periodic only.
     */
    public void coastMode(){
        m_turretMotor.setNeutralMode(NeutralMode.Coast);
        m_feederMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * called periodically to update our flywheel gains when we change them on the shuffleboard
     */
    private void setGains(){
        //Boolean to track whether any of the gains have changed since the last cycle
        boolean anythingChanged = false;
        //If the gains changed in shuffleboard, change them in our code
        if(m_masterConfig.slot2.kP != m_currentKP){
            anythingChanged = true;
            m_masterConfig.slot2.kP = m_currentKP; // Our original gain: 0.57
        }
        
        if(m_masterConfig.slot2.kI != m_currentKI){
            anythingChanged = true;
            m_masterConfig.slot2.kI = m_currentKI; // Our original gain: 0
        }
        
        if(m_masterConfig.slot2.kD != m_currentKD){
            anythingChanged = true;
            m_masterConfig.slot2.kD = m_currentKD; // Our original gain: 16
        }

        if(m_masterConfig.slot2.kF != m_currentKF){
            anythingChanged = true;
            m_masterConfig.slot2.kF = m_currentKF; // Our original gain: 0.05
        }
        
        //if any of the gains were changed, reconfigure the PID with the new settings
        if(anythingChanged){
            m_masterFlywheelMotor.configAllSettings(m_masterConfig);
        }
        
    }

    /**
     * Called in initialization to apply all of our motor configurations for PID on the flywheel
     */
    private void configTalonPID(){
        m_masterFlywheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
        m_masterConfig.slot2.kP = m_currentKP; //Original: 0.57 //Danny's: 0.55
        m_masterConfig.slot2.kI = m_currentKI; //Original: 0 //Danny's: 0.005
        m_masterConfig.slot2.kD = m_currentKD; //Original: 16 //Danny's: 13
        m_masterConfig.slot2.kF = m_currentKF; //Original: 0.05 //Danny's: 0.045
        m_masterConfig.slot2.integralZone = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kIzone;
		m_masterConfig.slot2.closedLoopPeakOutput = RobotMap.LauncherConstants.FLYWHEEL_GAINS.kPeakOutput;
        m_masterConfig.slot2.closedLoopPeriod = 1;
        m_masterConfig.slot2.allowableClosedloopError = 0;
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