package frc.robot;

// Imports for our shuffleboard so we can make new tabs and entries for them
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotShuffleboard {
    Launcher m_launcher;
    // Declares the member variable to later create the driver tab to control the scalars in the shuffleboard tab
    ShuffleboardTab m_driverTab;

    ShuffleboardTab m_PidTab;
    // Declares the member variables for the scalars
    double m_highVelocityScalar;
    double m_lowVelocityScalar;
    double m_highTurnScalar;
    double m_lowTurnScalar;
    // Declares member variables for manual launcher testing
    double m_flywheelVelocity;
    double m_targetFlywheelSpeed;
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

    double m_currentRpm;
    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityScalarEntry;
    private NetworkTableEntry m_lowVelocityScalarEntry;
    private NetworkTableEntry m_highTurnScalarEntry;
    private NetworkTableEntry m_lowTurnScalarEntry;
    private NetworkTableEntry m_flywheelVelocityEntry;
    private NetworkTableEntry m_autonPathEntry;
    private NetworkTableEntry m_launchPresetEntry;
    private NetworkTableEntry m_targetFlywheelSpeedEntry;
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
    public RobotShuffleboard(Launcher launcher){
        m_launcher = launcher;
        // Creates the new tab on the Shuffleboard for the driver
        m_driverTab = Shuffleboard.getTab("Driver Tab");
        m_PidTab = Shuffleboard.getTab("PID Tab");
    }

    /**
     * Initialization method for RobotShuffleBoard
     * Calls drivetrainShuffleboardConfig, setDrivetrainInputScalar, and setFlywheelVelocity to create the shuffleboard and its inputs.
     * Also sets the current values for inputs to the values from the shuffleboard.
     */
    public void init(){
        drivetrainShuffleboardConfig();
        PidShuffleboardConfig();
        setDrivetrainInputScalar();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelSpeed();
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
        
        setDrivetrainInputScalar();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelSpeed();
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
        m_highVelocityScalarEntry = m_driverTab.addPersistent("High Gear Speed Scalar", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowVelocityScalarEntry = m_driverTab.addPersistent("Low Gear Speed Scalar", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_highTurnScalarEntry = m_driverTab.addPersistent("High Gear Turn Scalar", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowTurnScalarEntry = m_driverTab.addPersistent("Low Gear Turn Scalar", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_flywheelVelocityEntry = m_driverTab.addPersistent("Flywheel Velocity", RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_autonPathEntry = m_driverTab.addPersistent("Auton Path", RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_launchPresetEntry = m_driverTab.addPersistent("Launch Preset", RobotMap.ShuffleboardConstants.DEFAULT_LAUNCH_PRESET)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_maxTurretSpeedEntry = m_driverTab.addPersistent("Max Turret Speed", RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_proportionalConstantEntry = m_driverTab.addPersistent("Proportional Constant", RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT)
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
        m_targetFlywheelSpeedEntry = m_PidTab.addPersistent("Target Flywheel Speed", RobotMap.LauncherConstants.TARGET_FLYWHEEL_SPEED)
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
        m_PidTab.addPersistent("current RPM PID", m_launcher.getRealSpeed())
                            .withWidget(BuiltInWidgets.kTextView)
                            .getEntry();
    }

    /**
     * Sets the input scalars for velocity and turning as the values found on the Shuffleboard
     */
    private void setDrivetrainInputScalar(){
        // Sets highVelocityScalar equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highVelocityScalar = m_highVelocityScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        // Sets lowVelocityScalar equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowVelocityScalar = m_lowVelocityScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        // Sets highTurnScalar equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highTurnScalar = m_highTurnScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        // Sets lowTurnScalar equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowTurnScalar = m_lowTurnScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
    }

    /**
     * Sets the velocity of the flywheel equal to the value on the shuffleboard. 
     * If no value is found on the shuffleboard for flywheelVelocity, returns with nothing, the value will be set to 0.5 from RobotMap
     */
    private void setFlywheelVelocity(){
        m_flywheelVelocity = m_flywheelVelocityEntry.getDouble(RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY);
    }

    /**
     * Gets the current value for the flywheel velocity from the shuffleboard
     * @return the flywheel velocity double from the shuffleboard
     */
    public double getFlywheelVelocity(){
        // Gets the updated value from the shuffleboard
        setFlywheelVelocity();
        return m_flywheelVelocity;
    }

    /**
     * Sets the targetFlywheelSpeed to be equal to the value on the shuffleboard
     * If no valued is found on the shuffleboard for targetFlywheelSpeed, meaning returns with nothing, the value will be set to 2,124 RPM
     */
    private void setTargetFlywheelSpeed(){
        m_targetFlywheelSpeed = m_targetFlywheelSpeedEntry.getDouble(RobotMap.LauncherConstants.TARGET_FLYWHEEL_SPEED);
        m_launcher.setTargetFlywheelSpeed(m_targetFlywheelSpeed);
    }

    /**
     * Gets the current target flywheel speed from the shuffleboard
     * @return the speed in RPM that we want the flywheels to be going before launching
     */
    public double getTargetFlywheelSpeed(){
        setTargetFlywheelSpeed();
        return m_targetFlywheelSpeed;
    }

    /**
     * Sets maxTurretSpeed and proportional constant equal to value input on the shuffleboard
     */
    private void setTurretValues(){
        m_maxTurretSpeed = m_maxTurretSpeedEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED);
        m_proportionalConstant = m_proportionalConstantEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT);
        m_launcher.setMaxTurretSpeed(m_maxTurretSpeed);
        m_launcher.setProportionalConstant(m_proportionalConstant);
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
     * Gets the value for HighVelocityScalar and returns it
     * @return The value of HighVelocity Scalar
     */
    public double getHighVelocityScalar(){
        setDrivetrainInputScalar();
        return m_highVelocityScalar;
    }

    /**
     * Gets the value for LowVelocityScalar and returns it
     * @return The value for LowVelocityScalar
     */
    public double getLowVelocityScalar(){
        setDrivetrainInputScalar();
        return m_lowVelocityScalar;
    }

    /**
     * Gets the value for HighTurnScalar and returns it
     * @return The value for HighTurnScalar
     */
    public double getHighTurnScalar(){
        setDrivetrainInputScalar();
        return m_highTurnScalar;
    }

    /**
     * Gets the value for LowTurnScalar and returns it
     * @return The value for LowTurnScalar
     */
    public double getLowTurnScalar(){
        setDrivetrainInputScalar();
        return m_lowTurnScalar;
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
