package frc.robot;

// Imports for our shuffleboard so we can make new tabs and entries for them
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotShuffleboard {
    Launcher m_launcher;
    LimelightVision m_limelight;
    // Declares the member variable to later create the driver tab to control the scalers in the shuffleboard tab
    ShuffleboardTab m_driverTab;

    ShuffleboardTab m_PidTab;
    // Declares the member variables for the scalers
    double m_highVelocityScaler;
    double m_lowVelocityScaler;
    double m_highTurnScaler;
    double m_lowTurnScaler;
    // Declares member variables for manual launcher testing
    double m_flywheelPercentPower;
    double m_targetFlywheelRpm;
    // Declares member variable for choosing an auton path
    double m_autonPath;
    double m_launchPreset;
    // Declares member variables for max turret speed and proportional constant
    double m_maxTurretSpeed;
    double m_proportionalConstant;

    double m_launcherKP;
    double m_launcherKI;
    double m_launcherKD;
    double m_launcherKF;

    boolean m_rpmBoolean;

    double m_currentRpm;
    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityScalerEntry;
    private NetworkTableEntry m_lowVelocityScalerEntry;
    private NetworkTableEntry m_highTurnScalerEntry;
    private NetworkTableEntry m_lowTurnScalerEntry;
    private NetworkTableEntry m_flywheelPercentPowerEntry;
    private NetworkTableEntry m_autonPathEntry;
    private NetworkTableEntry m_launchPresetEntry;
    private NetworkTableEntry m_targetFlywheelRpmEntry;
    private NetworkTableEntry m_maxTurretSpeedEntry;
    private NetworkTableEntry m_proportionalConstantEntry;
    private NetworkTableEntry m_launcherKPEntry;
    private NetworkTableEntry m_launcherKIEntry;
    private NetworkTableEntry m_launcherKDEntry;
    private NetworkTableEntry m_launcherKFEntry;
    private NetworkTableEntry m_currentRpmEntry;

    /**
     * Constructor for robot shuffleboard class
     * Ceeates a the Driver Tab on the shuffleboard
     */
    public RobotShuffleboard(Launcher launcher, LimelightVision limelight){
        m_launcher = launcher;
        m_limelight = limelight;
        // Creates the new tab on the Shuffleboard for the driver
        m_driverTab = Shuffleboard.getTab("Driver Tab");
        m_PidTab = Shuffleboard.getTab("PID Tab");
    }

    /**
     * Initialization method for RobotShuffleBoard
     * Calls drivetrainShuffleboardConfig, setDrivetrainInputScaler, and setFlywheelVelocity to create the shuffleboard and its inputs.
     * Also sets the current values for inputs to the values from the shuffleboard.
     */
    public void init(){
        drivetrainShuffleboardConfig();
        PidShuffleboardConfig();
        setDrivetrainInputScaler();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelRpm();
        setTurretValues();
        setKP();
        setKI();
        setKD();
        setKF();
    }

    /**
     * This method should be called periodically in Teleop
     * This updates the values we are getting from the shuffleboard to be current
     */
    public void periodic(){
        SmartDashboard.putNumber("Current RPM", m_launcher.getRealSpeed());
        SmartDashboard.putNumber("Target RPM", getTargetFlywheelSpeed());
        SmartDashboard.putNumber("Current Voltage", m_launcher.getCurrentPercent());
        SmartDashboard.putBoolean("At RPM", m_launcher.m_atRPM);
        
        setDrivetrainInputScaler();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelRpm();
        setTurretValues();
        setKP();
        setKI();
        setKD();
        setKF();
    }

    /**
     * Configures shuffleboard so we can get the entries from the shuffleboard driver tab
     */
    public void drivetrainShuffleboardConfig(){
        // It selects the driver tab
        Shuffleboard.selectTab("Driver Tab");

        // Creates the entry for the driver tab and allows us to access it and set it equal our member variable we made fo each entry
        m_highVelocityScalerEntry = m_driverTab.addPersistent("High Gear Speed Scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowVelocityScalerEntry = m_driverTab.addPersistent("Low Gear Speed Scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_highTurnScalerEntry = m_driverTab.addPersistent("High Gear Turn Scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowTurnScalerEntry = m_driverTab.addPersistent("Low Gear Turn Scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_flywheelPercentPowerEntry = m_driverTab.addPersistent("Flywheel Percent Power", RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_PERCENT_POWER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_autonPathEntry = m_driverTab.addPersistent("Auton Path", RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_maxTurretSpeedEntry = m_driverTab.addPersistent("Max Turret Speed", RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
    }

    public void PidShuffleboardConfig(){
        Shuffleboard.selectTab("PID Tab");

        m_launcherKPEntry = m_PidTab.addPersistent(("kP"), RobotMap.LauncherConstants.FLYWHEEL_GAINS.kP)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_launcherKIEntry = m_PidTab.addPersistent(("kI"), RobotMap.LauncherConstants.FLYWHEEL_GAINS.kI)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_launcherKDEntry = m_PidTab.addPersistent(("kD"), RobotMap.LauncherConstants.FLYWHEEL_GAINS.kD)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_launcherKFEntry = m_PidTab.addPersistent(("kF"), RobotMap.LauncherConstants.FLYWHEEL_GAINS.kF)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_targetFlywheelRpmEntry = m_PidTab.addPersistent("Target Flywheel Rpm", RobotMap.LauncherConstants.TARGET_FLYWHEEL_RPM)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_PidTab.addPersistent("current RPM PID", m_launcher.getRealSpeed())
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();

        m_launchPresetEntry = m_PidTab.addPersistent(("Launch Preset"), RobotMap.ShuffleboardConstants.DEFAULT_LAUNCH_PRESET)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_maxTurretSpeedEntry = m_PidTab.addPersistent(("Max Turret Speed"), RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_proportionalConstantEntry = m_PidTab.addPersistent(("Proportional Constant"), RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
    }

    /**
     * Sets the input Scalers for velocity and turning as the values found on the Shuffleboard
     */
    private void setDrivetrainInputScaler(){
        // Sets highVelocityScaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highVelocityScaler = m_highVelocityScalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets lowVelocityScaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowVelocityScaler = m_lowVelocityScalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets highTurnScaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highTurnScaler = m_highTurnScalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets lowTurnScaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowTurnScaler = m_lowTurnScalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
    }

    /**
     * Sets the velocity of the flywheel equal to the value on the shuffleboard. 
     * If no value is found on the shuffleboard for flywheelVelocity, returns with nothing, the value will be set to 0.5 from RobotMap
     */
    private void setFlywheelVelocity(){
        m_flywheelPercentPower = m_flywheelPercentPowerEntry.getDouble(RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_PERCENT_POWER);
        m_launcher.setFlywheelPercentPower(m_flywheelPercentPower);
    }

    /**
     * Gets the current value for the flywheel velocity from the shuffleboard
     * @return the flywheel velocity double from the shuffleboard
     */
    public double getFlywheelVelocity(){
        // Gets the updated value from the shuffleboard
        setFlywheelVelocity();
        return m_flywheelPercentPower;
    }

    /**
     * Sets the targetFlywheelSpeed to be equal to the value on the shuffleboard
     */
    private void setTargetFlywheelRpm(){
        m_targetFlywheelRpm = m_targetFlywheelRpmEntry.getDouble(RobotMap.LauncherConstants.TARGET_FLYWHEEL_RPM);
        m_launcher.setTargetFlywheelRpm(m_targetFlywheelRpm);
    }

    /**
     * Gets the current target flywheel speed from the shuffleboard
     * @return the speed in RPM that we want the flywheels to be going before launching
     */
    public double getTargetFlywheelSpeed(){
        setTargetFlywheelRpm();
        return m_targetFlywheelRpm;
    }

    /**
     * Sets maxTurretSpeed and proportional constant equal to value input on the shuffleboard
     */
    private void setTurretValues(){
        m_maxTurretSpeed = m_maxTurretSpeedEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED);
        m_proportionalConstant = m_proportionalConstantEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT);
        m_launcher.setMaxTurretSpeed(m_maxTurretSpeed);
    }

    public double getTurretValues(){
        setTurretValues();
        return m_maxTurretSpeed;
    }
    
    private void setAutonPath(){
        m_autonPath = m_autonPathEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH);
    }

    public double getAutonPath(){
        setAutonPath();
        return m_autonPath;
    }

    private void setLaunchPreset(){
        m_launchPreset = m_launchPresetEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_LAUNCH_PRESET);
    }

    public double getLaunchPreset(){
        setLaunchPreset();
        return m_launchPreset;
    }
    

    /**
     * Gets the value for HighVelocityScaler and returns it
     * @return The value of HighVelocity Scaler
     */
    public double getHighVelocityScaler(){
        setDrivetrainInputScaler();
        return m_highVelocityScaler;
    }

    /**
     * Gets the value for LowVelocityScaler and returns it
     * @return The value for LowVelocityScaler
     */
    public double getLowVelocityScaler(){
        setDrivetrainInputScaler();
        return m_lowVelocityScaler;
    }

    /**
     * Gets the value for HighTurnScaler and returns it
     * @return The value for HighTurnScaler
     */
    public double getHighTurnScaler(){
        setDrivetrainInputScaler();
        return m_highTurnScaler;
    }

    /**
     * Gets the value for LowTurnScaler and returns it
     * @return The value for LowTurnScaler
     */
    public double getLowTurnScaler(){
        setDrivetrainInputScaler();
        return m_lowTurnScaler;
    }

    private void setKP(){
        m_launcherKP = m_launcherKPEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kP);
        m_launcher.setKP(m_launcherKP);
    }

    public double getKP(){
        setKP();
        return m_launcherKP;
    }

    private void setKI(){
        m_launcherKI = m_launcherKIEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kI);
        m_launcher.setKI(m_launcherKI);
    }
    
    public double getKI(){
        setKI();
        return m_launcherKI;
    }

    private void setKD(){
        m_launcherKD = m_launcherKDEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kD);
        m_launcher.setKD(m_launcherKD);
    }

    public double getKD(){
        setKD();
        return m_launcherKD;
    }

    private void setKF(){
        m_launcherKF = m_launcherKFEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kF);
        m_launcher.setKF(m_launcherKF);
    }

    public double getKF(){
        setKF();
        return m_launcherKF;
    }

}
