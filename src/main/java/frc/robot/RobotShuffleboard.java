package frc.robot;

// Imports for our shuffleboard so we can make new tabs and entries for them
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotShuffleboard {
    // Declares the member variable to later create the driver tab to control the scalers in the shuffleboard tab
    ShuffleboardTab m_driverTab;
    // Declares the member variables for the scalers
    double m_highVelocityscaler;
    double m_lowVelocityscaler;
    double m_highTurnscaler;
    double m_lowTurnscaler;
    // Declares member variables for manual launcher testing
    double m_flywheelVelocity;
    double m_targetFlywheelSpeed;
    // Declares member variable for choosing an auton path
    double m_autonPath;
    double m_launchPreset;
    // Declares member variables for max turret speed and proportional constant
    double m_maxTurretSpeed;
    double m_proportionalConstant;
    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityscalerEntry;
    private NetworkTableEntry m_lowVelocityscalerEntry;
    private NetworkTableEntry m_highTurnscalerEntry;
    private NetworkTableEntry m_lowTurnscalerEntry;
    private NetworkTableEntry m_flywheelVelocityEntry;
    private NetworkTableEntry m_autonPathEntry;
    private NetworkTableEntry m_launchPresetEntry;
    private NetworkTableEntry m_targetFlywheelSpeedEntry;
    private NetworkTableEntry m_maxTurretSpeedEntry;
    private NetworkTableEntry m_proportionalConstantEntry;

    /**
     * Constructor for robot shuffleboard class
     * Ceeates a the Driver Tab on the shuffleboard
     */
    public RobotShuffleboard(){
        // Creates the new tab on the Shuffleboard for the driver
        m_driverTab = Shuffleboard.getTab("Driver Tab");
    }

    /**
     * Initialization method for RobotShuffleBoard
     * Calls drivetrainShuffleboardConfig, setDrivetrainInputscaler, and setFlywheelVelocity to create the shuffleboard and its inputs.
     * Also sets the current values for inputs to the values from the shuffleboard.
     */
    public void init(){
        drivetrainShuffleboardConfig();
        setDrivetrainInputscaler();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelSpeed();
        setTurretValues();
    }

    /**
     * This method should be called periodically in Teleop
     * This updates the values we are getting from the shuffleboard to be current
     */
    public void periodic(){
        setDrivetrainInputscaler();
        setFlywheelVelocity();
        setAutonPath();
        setLaunchPreset();
        setTargetFlywheelSpeed();
        setTurretValues();
    }

    /**
     * Configures shuffleboard so we can get the entries from the shuffleboard driver tab
     */
    public void drivetrainShuffleboardConfig(){
        // It selects the driver tab
        Shuffleboard.selectTab("Driver Tab");

        // Creates the entry for the driver tab and allows us to access it and set it equal our member variable we made fo each entry
        m_highVelocityscalerEntry = m_driverTab.addPersistent("High Gear Speed scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowVelocityscalerEntry = m_driverTab.addPersistent("Low Gear Speed scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_highTurnscalerEntry = m_driverTab.addPersistent("High Gear Turn scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_lowTurnscalerEntry = m_driverTab.addPersistent("Low Gear Turn scaler", RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER)
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
        m_targetFlywheelSpeedEntry = m_driverTab.addPersistent("Target Flywheel Speed", RobotMap.LauncherConstants.TARGET_FLYWHEEL_SPEED)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_maxTurretSpeedEntry = m_driverTab.addPersistent("Max Turret Speed", RobotMap.ShuffleboardConstants.DEFAULT_MAX_TURRET_SPEED)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
        m_proportionalConstantEntry = m_driverTab.addPersistent("Proportional Constant", RobotMap.ShuffleboardConstants.DEFAULT_PROPORTIONAL_CONSTANT)
                                    .withWidget(BuiltInWidgets.kTextView)
                                    .getEntry();
    }

    /**
     * Sets the input scalers for velocity and turning as the values found on the Shuffleboard
     */
    private void setDrivetrainInputscaler(){
        // Sets highVelocityscaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highVelocityscaler = m_highVelocityscalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets lowVelocityscaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowVelocityscaler = m_lowVelocityscalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets highTurnscaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_highTurnscaler = m_highTurnscalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
        // Sets lowTurnscaler equal to the entry from the tab and if it returns with nothing sets it to default of 0.5
        m_lowTurnscaler = m_lowTurnscalerEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER);
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
     * Gets the value for HighVelocityscaler and returns it
     * @return The value of HighVelocity scaler
     */
    public double getHighVelocityscaler(){
        setDrivetrainInputscaler();
        return m_highVelocityscaler;
    }

    /**
     * Gets the value for LowVelocityscaler and returns it
     * @return The value for LowVelocityscaler
     */
    public double getLowVelocityscaler(){
        setDrivetrainInputscaler();
        return m_lowVelocityscaler;
    }

    /**
     * Gets the value for HighTurnscaler and returns it
     * @return The value for HighTurnscaler
     */
    public double getHighTurnscaler(){
        setDrivetrainInputscaler();
        return m_highTurnscaler;
    }

    /**
     * Gets the value for LowTurnscaler and returns it
     * @return The value for LowTurnscaler
     */
    public double getLowTurnscaler(){
        setDrivetrainInputscaler();
        return m_lowTurnscaler;
    }

}
