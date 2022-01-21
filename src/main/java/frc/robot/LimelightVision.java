package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightVision {
    public enum Pipeline{
        kStandard(0),
        kZoomX2(1),
        kZoomX3(2),
        kDriver(3);

        int m_pipelineID;

        private Pipeline(int pipelineID){
            m_pipelineID = pipelineID;
        }

        public int getID() {
            return m_pipelineID;
        }
    }

    public Pipeline m_Pipeline;

    // Declaration of the network table so values for m_xAngleOffset, m_yAngleOffset, and distance can be created
    NetworkTable m_limelightTable;

    // Horizontal offset from Crosshair to target (LL1: -27 to 27 degrees; LL2 -29.8 to 29.8 degrees)
    NetworkTableEntry m_tableXOffset;

    // Vertical offset from crosshair to target (LL1: -20.5 to 20.5 degrees; LL2: -24.85 to 24.85 degrees)
    NetworkTableEntry m_tableYOffset;

    // Target Area (0% to 100% of image)
    NetworkTableEntry m_tableScreenArea;

    double m_xAngleOffset;
    double m_yAngleOffset;
    double m_areaOfScreen;

    public LimelightVision (){
        m_limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");

        // assigns the variable as all the possible entries
        m_tableXOffset = m_limelightTable.getEntry("tx");
        m_tableYOffset = m_limelightTable.getEntry("ty");
        m_tableScreenArea = m_limelightTable.getEntry("ta"); 

        // gets the offset values for x and y direction and the area of the image that is within the camera's frame
        m_xAngleOffset = m_tableXOffset.getDouble(0.0);
        m_yAngleOffset = m_tableYOffset.getDouble(0.0);
        m_areaOfScreen = m_tableScreenArea.getDouble(0.0);

        // Creates/assigns the offset and area variables onto the dashboard table
        SmartDashboard.putNumber("LimelightX Offset", m_xAngleOffset);
        SmartDashboard.putNumber("LimelightY Offset", m_yAngleOffset);
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
        m_limelightTable.getEntry("ledMode").setDouble(1d);
    }

    /**
     * Restores the LED to pipeline control
     */
    public void enableLEDs() {
        m_limelightTable.getEntry("ledMode").setDouble(0d);
    }


    /**
     * Tells if the target is visible or not
     * @return true if we can see the target, false if we cannot
     */
    public boolean seeTarget(){
        return(m_limelightTable.getEntry("tv").getBoolean(false));
    }

    /**
     * Tells how far off to the left or right the target
     * @return It returns the angular value in the x direction from the target from -29.8 to 29.8
     */

    public double xAngleToTarget(){
        return(m_xAngleOffset);
    }

    /**
     * Tells how far off in the y direction we are from the target
     * @return The angular value of the y direction offset from the target (-24.85 to 24.85)
     */
    public double yAngleToTarget(){
        return(m_yAngleOffset);
    }
    /**
     * Tells the percent of the screen that the target takes up 
     * @return The percentage of the screen that the target takes up (0 to 1)
     */
    public double targetPercentVisible(){
        return(m_areaOfScreen);
    }

    /**
     * Finds the distance from the limelight to the top of the upper hub
     * @return The distance in inches
     */
    public double distToTarget(double cameraDegreesFromGround){
        //member variables to use in calculations
        double m_distance;
        double m_sineOfAngle;
        double m_totalHeight;
        //math to find the height from the camera to the top of the hub
        m_totalHeight = RobotMap.LimelightConstants.HUB_HEIGHT - RobotMap.LimelightConstants.CAMERA_HEIGHT;
        //finds the sine of the sum of both angles and sets it to one member variable
        m_sineOfAngle = Math.sin(cameraDegreesFromGround + yAngleToTarget());
        //calculates the distance of the hypotenuse (distance from camera to upper hub)
        m_distance = m_totalHeight/m_sineOfAngle;
        //returns calculated distance in inches
        return m_distance;
    }

}