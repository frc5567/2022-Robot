package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
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
    double m_xAngleOffset;

    // Vertical offset from crosshair to target (LL1: -20.5 to 20.5 degrees; LL2: -24.85 to 24.85 degrees)
    double m_yAngleOffset;

    // Target Area (0% to 100% of image)
    double m_areaOfScreen;
    
    //Constructor for limelight
    public LimelightVision (){
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

        // assigns and gets the variable for each entry
        m_xAngleOffset = m_limelightTable.getEntry("tx").getDouble(0.0);
        m_yAngleOffset = m_limelightTable.getEntry("ty").getDouble(0.0);
        m_areaOfScreen = m_limelightTable.getEntry("ta").getDouble(0.0); 

        // Creates/assigns the offset and area variables onto the dashboard table
        SmartDashboard.putNumber("LimelightX Offset", m_xAngleOffset);
        SmartDashboard.putNumber("LimelightY Offset", m_yAngleOffset);
        SmartDashboard.putNumber("LimelightArea Percentage", m_areaOfScreen);

    }

    public void limelightInit(){
        disableLEDs();
        setPipeline(Pipeline.kStandard);
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
        //NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode"); 
        //double value = (double)ledMode.getNumber(0);
        //System.out.println(value);
    }

    /**
     * Force the LED to turn on
     */
    public void enableLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        //NetworkTableEntry ledMode = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode"); 
        //double value = (double)ledMode.getNumber(0);
        //System.out.println(value);
    }


    /**
     * Tells if the target is visible or not
     * @return true if we can see the target, false if we cannot
     */
    public boolean seeTarget(){
        boolean returnVal = false;
        double retFromTable = (double)m_limelightTable.getEntry("tv").getNumber(0);
        if (retFromTable == 1){
            returnVal = true;
        }
        return returnVal;
    }

    /**
     * Tells how far off to the left or right the target
     * @return It returns the angular value in the x direction from the target from -29.8 to 29.8
     */
    public double xAngleToTarget(){
        return(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
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
    private double targetPercentVisible(){
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
        m_sineOfAngle = Math.sin(RobotMap.LimelightConstants.CAMERA_DEGREES_FROM_GROUND + yAngleToTarget());
        //calculates the distance of the hypotenuse (distance from camera to upper hub)
        m_distance = m_totalHeight/m_sineOfAngle;
        //returns calculated distance in inches
        return m_distance;
    }
  
    /**
     * Figures out where in the x direction (right or left) to go in terms of speed 
     * @return The speed for turning to the target
     */
    public double turnAngleAdjustToTargetSpeed(){
        // Sets initial speed to 0
        double m_xTurnSpeed = 0.0;
        // Checks to see if we are at an acceptable offset from the target
        if (xAngleToTarget() > 0.5){
            // If our offset is greater than acceptable maximum we set speed to negative to turn left until in acceptable range
            m_xTurnSpeed = -RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED;
    
        }
        else if (xAngleToTarget() < -0.5){
            // If our offset is less than acceptable minimmum we set speed to positive to turn right until in acceptable range
            m_xTurnSpeed = RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED;
        }
        return m_xTurnSpeed;
    }
  
    /**
     * Figures out where in the y direction to go (forward or backward) to go in terms of speed
     * @return The speed for moving to the target
     */
    public double distanceAdjustToTargetSpeed(){
        // Sets initial speed to 0
        double m_distanceAdjustSpeed = 0.0;
        // Checks to see if we are at an acceptable offset distance from the target
        if (yAngleToTarget() > 0.5){
            // If our distance offset is greater than the acceptable range we set the speed to move backwards
            m_distanceAdjustSpeed = -RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED;
        }
        else if (yAngleToTarget() < -0.5){
            // If our distance offset is less than than the acceptable range we set the speed to move forwards
            m_distanceAdjustSpeed = RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED;
        }
        return m_distanceAdjustSpeed;
    }

    /**
     * Periodically updates the x and y angle offsets and the area of the screen taken up by the target
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

}