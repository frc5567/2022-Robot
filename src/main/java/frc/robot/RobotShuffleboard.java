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
    // Declares member variables for the entries on the table
    private NetworkTableEntry m_highVelocityScalarEntry;
    private NetworkTableEntry m_lowVelocityScalarEntry;
    private NetworkTableEntry m_highTurnScalarEntry;
    private NetworkTableEntry m_lowTurnScalarEntry;

    /**
     * Initialization method for RobotShuffleBoard
     */
    public void initRobotShuffleboard(){
        drivetrainShuffleboardConfig();
        setDrivetrainInputScalar();
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
    }

    private void setDrivetrainInputScalar(){
        // Sets highVelocityScalar equal to the entry from the tab and if it returns with nothing sets it to default
        m_highVelocityScalar = m_highVelocityScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        m_lowVelocityScalar = m_lowVelocityScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        m_highTurnScalar = m_highTurnScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
        m_lowTurnScalar = m_lowTurnScalarEntry.getDouble(RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR);
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
