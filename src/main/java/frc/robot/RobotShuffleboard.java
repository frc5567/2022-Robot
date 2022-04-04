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
    // Declares the member variable for the driver tab to control the scalers in the shuffleboard 
    ShuffleboardTab m_driverTab;
    // Declares the member variable for the PID tab to control PID values in the shuffleboard
    ShuffleboardTab m_PidTab;
    // Declares the member variables for the scalers
    double m_highVelocityScaler;
    double m_lowVelocityScaler;
    double m_highTurnScaler;
    double m_lowTurnScaler;
    // Declares member variables for flywheel control
    double m_flywheelPercentPower;
    double m_targetFlywheelRpm;
    // Declares member variable for choosing an auton path
    double m_autonPath;
    // Declares member variable for setting the max turret speed
    double m_maxTurretSpeed;
    // Declares member variables to set the gains on the flywheel from the shuffleboard
    double m_launcherKP;
    double m_launcherKI;
    double m_launcherKD;
    double m_launcherKF;

    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityScalerEntry;
    private NetworkTableEntry m_lowVelocityScalerEntry;
    private NetworkTableEntry m_highTurnScalerEntry;
    private NetworkTableEntry m_lowTurnScalerEntry;
    private NetworkTableEntry m_flywheelPercentPowerEntry;
    private NetworkTableEntry m_autonPathEntry;
    //Commented out because it is currently unused, but should be used if we want to do further distance testing
    //private NetworkTableEntry m_targetFlywheelRpmEntry;
    private NetworkTableEntry m_maxTurretSpeedEntry;
    private NetworkTableEntry m_launcherKPEntry;
    private NetworkTableEntry m_launcherKIEntry;
    private NetworkTableEntry m_launcherKDEntry;
    private NetworkTableEntry m_launcherKFEntry;

    /**
     * Constructor for robot shuffleboard class
     * Sets launcher and limelight objects and initializes both tabs
     */
    public RobotShuffleboard(Launcher launcher, LimelightVision limelight){
        m_launcher = launcher;
        m_limelight = limelight;
        // Initializes the new tab on the Shuffleboard for drive values
        m_driverTab = Shuffleboard.getTab("Driver Tab");
        // Initializes a new tab on the shuffleboard for PID values
        m_PidTab = Shuffleboard.getTab("PID Tab");
    }

    /**
     * Initialization method for RobotShuffleBoard, called every time the robot is enabled
     * Configures the shuffleboard tabs and sets all of the values from shuffleboard
     */
    public void init(){
        drivetrainShuffleboardConfig();
        PidShuffleboardConfig();
        setDrivetrainInputScaler();
        setFlywheelVelocity();
        setAutonPath();
        //Commented out because it is currently unused, but should be used if we want to do further distance testing
        //setTargetFlywheelRpm();
        setMaxTurretSpeed();
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
        //Commented out because it is currently unused, but should be used if we want to do further distance testing
        // SmartDashboard.putNumber("Target RPM", getTargetFlywheelRpm());
        SmartDashboard.putNumber("Current Voltage", m_launcher.getCurrentPercent());
        SmartDashboard.putBoolean("At RPM", m_launcher.m_atRPM);
        
        setDrivetrainInputScaler();
        setFlywheelVelocity();
        setAutonPath();
        //Commented out because it is currently unused, but should be used if we want to do further distance testing
        //setTargetFlywheelRpm();
        setMaxTurretSpeed();
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

    /**
     * Configures shuffleboard so we can get the entries from the PID driver tab
     */
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
        //Commented out because it is currently unused, but should be used if we want to do further distance testing
        // m_targetFlywheelRpmEntry = m_PidTab.addPersistent("Target Flywheel Rpm", RobotMap.LauncherConstants.DEFAULT_TARGET_FLYWHEEL_RPM)
        //                     .withWidget(BuiltInWidgets.kTextView)
        //                     .getEntry();
        m_PidTab.addPersistent("current RPM PID", m_launcher.getRealSpeed())
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();

        m_maxTurretSpeedEntry = m_PidTab.addPersistent(("Max Turret Speed"), RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED)
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
     * Gets the value for HighVelocityScaler from the shuffleboard
     * @return The value of HighVelocity Scaler
     */
    public double getHighVelocityScaler(){
        setDrivetrainInputScaler();
        return m_highVelocityScaler;
    }

    /**
     * Gets the value for LowVelocityScaler from the shuffleboard
     * @return The value for LowVelocityScaler
     */
    public double getLowVelocityScaler(){
        setDrivetrainInputScaler();
        return m_lowVelocityScaler;
    }

    /**
     * Gets the value for HighTurnScaler from the shuffleboard
     * @return The value for HighTurnScaler
     */
    public double getHighTurnScaler(){
        setDrivetrainInputScaler();
        return m_highTurnScaler;
    }

    /**
     * Gets the value for LowTurnScaler from the shuffleboard
     * @return The value for LowTurnScaler
     */
    public double getLowTurnScaler(){
        setDrivetrainInputScaler();
        return m_lowTurnScaler;
    }

    /**
     * Records the current Auton Path from the shuffleboard
     * If no value is found on the shuffleboard for autonPath, the value will be set to 2 (two ball)
     */
    private void setAutonPath(){
        m_autonPath = m_autonPathEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH);
    }

    /**
     * Gets the current Auton path from the shuffleboard
     * @return the path the robot will take in auton
     */
    public double getAutonPath(){
        setAutonPath();
        return m_autonPath;
    }

    /**
     * Records the percent power of the value on the shuffleboard and sets it in the Launcher class
     * If no value is found on the shuffleboard for flywheelVelocity, the value will be set to 0.72
     */
    private void setFlywheelVelocity(){
        m_flywheelPercentPower = m_flywheelPercentPowerEntry.getDouble(RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_PERCENT_POWER);
        m_launcher.setFlywheelPercentPower(m_flywheelPercentPower);
    }

    /**
     * Records the target flywheel RPM from the shuffleboard and sets it in the Launcher class
     * If no value is found on the shuffleboard for targetFlywheelRPM, the value will be set to 4000
     * Commented out because it is currently unused, but should be used if we want to do further distance testing
     */
    // private void setTargetFlywheelRpm(){
    //     m_targetFlywheelRpm = m_targetFlywheelRpmEntry.getDouble(RobotMap.LauncherConstants.DEFAULT_TARGET_FLYWHEEL_RPM);
    //     m_launcher.setTargetFlywheelRpm(m_targetFlywheelRpm);
    // }

    /**
     * Gets the current target flywheel RPM from the shuffleboard
     * Commented out because it is currently unused, but should be used if we want to do further distance testing
     * @return the speed in RPM that we want the flywheels to be going before launching
     */
    // public double getTargetFlywheelRpm(){
    //     setTargetFlywheelRpm();
    //     return m_targetFlywheelRpm;
    // }

    /**
     * Records the maximum turret percent power from the shuffleboard and sets it in the Launcher class
     * If no value is found on the shuffleboard for maxTurretSpeed, the value will be set to 0.75
     */
    private void setMaxTurretSpeed(){
        m_maxTurretSpeed = m_maxTurretSpeedEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED);
        m_launcher.setMaxTurretSpeed(m_maxTurretSpeed);
    }

    /**
     * Sets our P gain in the Launcher class to the value on shuffleboard
     * If no value is found in shuffleboard, set it to the P constant in RobotMap
     */
    private void setKP(){
        m_launcherKP = m_launcherKPEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kP);
        m_launcher.setKP(m_launcherKP);
    }

    /**
     * Sets our Igain in the Launcher class to the value on shuffleboard
     * If no value is found in shuffleboard, set it to the I constant in RobotMap
     */
    private void setKI(){
        m_launcherKI = m_launcherKIEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kI);
        m_launcher.setKI(m_launcherKI);
    }

    /**
     * Sets our D gain in the Launcher class to the value on shuffleboard
     * If no value is found in shuffleboard, set it to the D constant in RobotMap
     */
    private void setKD(){
        m_launcherKD = m_launcherKDEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kD);
        m_launcher.setKD(m_launcherKD);
    }

    /**
     * Sets our F gain in the Launcher class to the value on shuffleboard
     * If no value is found in shuffleboard, set it to the F constant in RobotMap
     */
    private void setKF(){
        m_launcherKF = m_launcherKFEntry.getDouble(RobotMap.LauncherConstants.FLYWHEEL_GAINS.kF);
        m_launcher.setKF(m_launcherKF);
    }
}
