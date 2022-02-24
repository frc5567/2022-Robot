package frc.robot;

// Imports for our shuffleboard so we can make new tabs and entries for them
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.networktables.NetworkTableEntry;

public class RobotShuffleboard {
    // Declares the member variable to later create the driver tab to control the scalars in the shuffleboard tab
    ShuffleboardTab m_driverTab;
    // Declares the member variables for the scalars
    double m_highVelocityScalar;
    double m_lowVelocityScalar;
    double m_highTurnScalar;
    double m_lowTurnScalar;
    // Declares member variable for manual launcher testing
    double m_flywheelVelocity;
    // Declares member variable for choosing an auton path
    double m_autonPath;
    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityScalarEntry;
    private NetworkTableEntry m_lowVelocityScalarEntry;
    private NetworkTableEntry m_highTurnScalarEntry;
    private NetworkTableEntry m_lowTurnScalarEntry;
    private NetworkTableEntry m_flywheelVelocityEntry;
    private NetworkTableEntry m_autonPathEntry;

    /**
     * Initialization method for RobotShuffleBoard
     * Calls drivetrainShuffleboardConfig, setDrivetrainInputScalar, and setFlywheelVelocity to create the shuffleboard and its inputs.
     * Also sets the current values for inputs to the values from the shuffleboard.
     */
    public void init(){
        drivetrainShuffleboardConfig();
        setDrivetrainInputScalar();
        setFlywheelVelocity();
        setAutonPath();
    }

    /**
     * This method should be called periodically in Teleop
     */
    public void periodic(){
        setDrivetrainInputScalar();
        setFlywheelVelocity();
        setAutonPath();
    }

 
    /**
     * Constructor for robot shuffleboard class
     */
    public RobotShuffleboard(){
        // Creates the new tab on the Shuffleboard for the driver
        m_driverTab = Shuffleboard.getTab("Driver Tab");
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
     * Sets the velocity of the flywheel equal to the value on the shuffleboard
     */
    private void setFlywheelVelocity(){
        m_flywheelVelocity = m_flywheelVelocityEntry.getDouble(RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY);
    }
    
    private void setAutonPath(){
        m_autonPath = m_autonPathEntry.getDouble(RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH);
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

}
