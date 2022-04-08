package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightVision {
    /**
     * Creates an enum to store the pipeline kStandard and the methond to get its ID
     */
    public enum Pipeline{
        // Declares kStandard as a mode for the pipeline for the way the image data is processed
        kStandard(0);

        int m_pipelineID;

        private Pipeline(int pipelineID){
            m_pipelineID = pipelineID;
        }

        public int getID() {
            return m_pipelineID;
        }
    }

    // Declares the pipeline object for the Limelight that processes the images into data that we need to find for targeting
    public Pipeline m_pipeline;

    // Declaration of the network table so values for m_xAngleOffset, m_yAngleOffset, and distance can be created
    NetworkTable m_limelightTable;

    // Declares object for horizontal offset from Crosshair to target (-29.8 to 29.8 degrees)
    double m_xAngleOffset;

    // Declares object for vertical offset from crosshair to target (-24.85 to 24.85 degrees)
    double m_yAngleOffset;

    // Declares object for Target Area (0% to 100% of image)
    double m_areaOfScreen;
    
    /**
     * Constructor for limelight to allow robot to target for launching
     * Creates SmartDashboard with offset values in the x and y directions and area of the screen taken up by the target that is found by the Limelight
     */
    public LimelightVision (){
        // Creates the Network Table that stores the values for the Limelight and allows those values to be easily changed for testing
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // assigns and gets the variable for each entry
        // tx is the horizontal distance between target angle and the center of the screen
        m_xAngleOffset = m_limelightTable.getEntry("tx").getDouble(0.0);
        // ty is the vertical distance between target angle and the center of the screen
        m_yAngleOffset = m_limelightTable.getEntry("ty").getDouble(0.0);
        // ta is the percent area of the screen that is taken up by the target
        m_areaOfScreen = m_limelightTable.getEntry("ta").getDouble(0.0); 

        // Creates/assigns the offset and area variables onto the dashboard table
        SmartDashboard.putNumber("LimelightX Offset", m_xAngleOffset);
        SmartDashboard.putNumber("LimelightY Offset", m_yAngleOffset);
        SmartDashboard.putNumber("LimelightArea Percentage", m_areaOfScreen);
    }

    /**
     * Limelight init to initially disable the LEDs and set the pipeline
     */
    public void init(){
        //disableLEDs();
        setPipeline(Pipeline.kStandard);
    }

    /**
     * Periodically updates the x and y angle offsets and the area of the screen taken up by the target. 
     * This is so we know we have the right values when we go to use them.
     */
    public void periodic(){
        // Updates the table entry for the x offset
        m_xAngleOffset = m_limelightTable.getEntry("tx").getDouble(0.0);
        // Updates the table entry for the y offset
        m_yAngleOffset = m_limelightTable.getEntry("ty").getDouble(0.0);
        // Updates the table entry for the area of the screen taken up by the target
        m_areaOfScreen = m_limelightTable.getEntry("ta").getDouble(0.0); 

        // Puts the x angle offset value on the shuffleboard
        SmartDashboard.putNumber("LimelightX Offset", m_xAngleOffset);
        // Puts the y angle offset value on the shuffleboard
        SmartDashboard.putNumber("LimelightY Offset", m_yAngleOffset);
        // Puts the percentage of the screen that the target takes up on the shuffleboard
        SmartDashboard.putNumber("LimelightArea Percentage", m_areaOfScreen);
    }
    
    /**
     * Sets pipeline number (0-9 value)
     * @param Pipeline Pipeline type to get ID number from
     */
    public static void setPipeline(Pipeline pipeline) {
        NetworkTableInstance.getDefault().getEntry("pipeline").setNumber(pipeline.getID());
    }

     /**
     * Forces the LEDs to turn off on the limelight
     */
    public void disableLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    /**
     * Force the limelight's LEDs to turn on
     */
    public void enableLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    /**
     * Current status of the LEDs
     * @return 1 if LEDs are off, 3 if LEDs are on
     */
    public int currentLEDStatus(){
        double currentValue = m_limelightTable.getEntry("ledMode").getDouble(0);
        //This makes sure that only a whole number can ever be called since the values for the LEDS being on are whole numbers.
        int intValue = (int)Math.round(currentValue);
        return intValue;
    }

    /**
     * Returns if the target is visible or not
     * @return true if we can see the target, false if we cannot
     */
    public boolean seeTarget(){
        boolean returnVal = false;
        // If tv (target visible) is a 1, target is visible. If tv is 0, the target is not visible (no value is found)
        double retFromTable = (double)m_limelightTable.getEntry("tv").getNumber(0);
        // Checks to see if the value from the Limelight is at the desired value of 1 or true, meaning the target is visible
        if (retFromTable == 1){
            returnVal = true;
        }
        return returnVal;
    }

    /**
     * Tells how far off to the left or right the target is
     * @return It returns the angular value in the x direction from the target (-29.8 to 29.8)
     */
    public double xAngleToTarget(){
        return(m_xAngleOffset);
    }

    /**
     * Tells how far off in the y direction the robotis is from the target
     * @return The angular value of the y direction offset from the target (-24.85 to 24.85)
     */
    public double yAngleToTarget(){
        return(m_yAngleOffset);
    }

    /**
     * This method is for determining the percent of the screen that the target takes up
     * @return the area of the screen taken up by the target (0% to 100%)
     */
    public double tAreaOfScreen(){
        return(m_areaOfScreen);
    }

    /**
     * Finds the distance from the limelight straight to the hub (parallel to the ground)
     * @return The distance in inches from the hub
     */
    public double distToTarget(){
        //variables to use in calculations
        double distance;
        double tanOfAngle;
        double totalHeight;
        //Finds the height from the camera to the top of the hub
        totalHeight = RobotMap.LimelightConstants.HUB_HEIGHT - RobotMap.LimelightConstants.CAMERA_HEIGHT;
        //finds the sine of the sum of both angles and sets it to one variable (tanOfAngle) for simplicity of the final calculation
        tanOfAngle = Math.tan((RobotMap.LimelightConstants.CAMERA_DEGREES_FROM_GROUND + m_yAngleOffset) * RobotMap.LimelightConstants.ANGLE_TO_RADIAN_CONVERT);
        //calculates the distance that the robot is from the hub (parallel to the ground)
        distance = totalHeight/tanOfAngle;
        //returns calculated distance in inches. Have to double the distance (multiply by 2) because what is returned is half of the actual distance if we don't.
        return distance * 2;
    }
}