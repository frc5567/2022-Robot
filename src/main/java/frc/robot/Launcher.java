package frc.robot;

// Import motor controllers
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;

public class Launcher{
    //Enum for storing the possible states of the launcher
    public enum LauncherState{
        // Idle state for the Launcher
        kIdle("Idle"),
        // Launch state for the Launcher
        kLaunch("Launch");

        private String stateName;

        LauncherState (String stateName){
            this.stateName = stateName;
        }

        //toString for the current state of the launcher
        public String toString(){
            return this.stateName;
        }
    }

    //Declares launcher state
    LauncherState m_state;
    
    //Declares variables for the motor that moves the launcher flywheel and the motor that controls the turret angle
    private TalonFX m_flywheelMotor;
    private TalonFX m_feederMotor;
    private TalonFX m_turretMotor;

    //Declares variables for the encoders that track the launcher flywheel and the turret angle
    private SensorCollection m_flywheelEncoder;
    private SensorCollection m_feederEncoder;
    private SensorCollection m_turretEncoder;

    /**
     * Constructor for Launcher objects
     */
    public Launcher(){
        m_flywheelMotor = new TalonFX(RobotMap.LauncherConstants.FLYWHEEL_FALCON_ID);
        m_feederMotor = new TalonFX(RobotMap.LauncherConstants.FEEDER_FALCON_ID);
        m_turretMotor = new TalonFX(RobotMap.LauncherConstants.TURRET_FALCON_ID);
    }

    
    /*
    //method for preparing the launch sequence
    This is commented out because we will need to impliment it into the copilot controller class and we don't have enough buttons 
    left on the pilot controller for testing. It also needs distance and angle calculations from the limelight for the targeting. 
    Once the copilot class exists, we will make it so when the button is released it returns to idle.
    public void prepareLaunch(){
        
        look for target
        turn to target
        
        setState(LauncherState.kLaunch);
    }

    //method for feeding the ball into the flywheel once it's revved up to speed
    public void launch(){
        setFlywheelSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
    }
    */

    //Zeros encoders
    public void zeroEncoders(){
        m_flywheelEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_feederEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
        m_turretEncoder.setQuadraturePosition(0, RobotMap.LauncherConstants.CONFIG_TIMEOUT_MS);
    }

    //Sets the state of the launcher flywheel
    public void setState(LauncherState launcherState){
        if(m_state == launcherState) {
            return;
        }
        //Sets the input state to our state
        m_state = launcherState;

        //Checks which state we are in and sets motor speed for each state
        if(m_state == LauncherState.kIdle) {
           setFlywheelSpeed(RobotMap.LauncherConstants.IDLE_SPEED);
        }
        else if (m_state == LauncherState.kLaunch){
            setFlywheelSpeed(RobotMap.LauncherConstants.FIRING_SPEED);
        }
    }

    //Sets the speed of the feeder motor
    public void setFeederSpeed(double speed){
        m_feederMotor.set(ControlMode.PercentOutput, speed);
    }

    //Sets the speed of the launcher flywheel motor
    public void setFlywheelSpeed(double speed){
        m_flywheelMotor.set(ControlMode.PercentOutput, speed);
    }

    //Returns the current state of the Launcher
    public LauncherState getState(){
        return m_state;
    }

    //Returns current speed of flywheel motor
    public double getRealSpeed(){
        return m_flywheelMotor.getSelectedSensorVelocity();
    }

    //Returns target speed of flywheel motor
    public double getCmdSpeed(){
        if (m_state == LauncherState.kLaunch){
            return RobotMap.LauncherConstants.FIRING_SPEED;
        }
        else {
            return (0.0);
        }
    }
}