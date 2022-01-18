package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightVision {
    // Declaration of the network table so values for m_x, m_y, and distance can be created
    NetworkTable m_limelightTable;

    // Horizontal offset from Crosshair to target (LL1: -27 to 27 degrees; LL2 -29.8 to 29.8 degrees)
    NetworkTableEntry m_tx;

    // Vertical offset from crosshair to target (LL1: -20.5 to 20.5 degrees; LL2: -24.85 to 24.85 degrees)
    NetworkTableEntry m_ty;

    // Target Area (0% to 100% of image)
    NetworkTableEntry m_ta;

    double m_x;
    double m_y;
    double m_area;

    public LimelightVision (){
        m_limelightTable = NetworkTableInstance.getDefault().getTable("Limelight");

        // assigns the variable as all the possible entries
        m_tx = m_limelightTable.getEntry("tx");
        m_ty = m_limelightTable.getEntry("ty");
        m_ta = m_limelightTable.getEntry("ta"); 

        // gets the offset values for x and y direction and the area of the image that is within the camera's frame
        m_x = m_tx.getDouble(0.0);
        m_y = m_ty.getDouble(0.0);
        m_area = m_ta.getDouble(0.0);

        // Creates/assigns the offset and area variables onto the dashboard table
        SmartDashboard.putNumber("LimelightX Offset", m_x);
        SmartDashboard.putNumber("LimelightY Offset", m_y);
        SmartDashboard.putNumber("LimelightArea Percentage", m_area);

    }

    //public double angleToTarget(){
        // 


    //}


}
