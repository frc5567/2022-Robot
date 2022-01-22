package frc.robot;

public class Auton{    
    //declares instances of our drivetrain and the auton step enum
    private AutonStep m_step;
    private AutonPath m_path;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;

    /**
     * constructor for auton
     * @param drivetrain drivetrain mechanism on the robot
     */
    public Auton(Drivetrain drivetrain){
        m_drivetrain = drivetrain;
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kLeftWall;
    }
    
    //enum for what path we are going to take in auton
    public enum AutonPath{
        //auton path for starting on the hub wall on the left side
        kLeftWall,

        //auton path for starting on the hub wall on the left side
        kRightWall,

        //auton path for starting on the edge of the tarmac line on the right side
        kRightLine;
    }

    //enum for each of the steps in our auton
    public enum AutonStep{
        kStep1,

        kStop;
    }

    //this method will be run at the start of every auton period
    public void init(){
        m_drivetrain.zeroEncoders();
        m_launcher.zeroEncoders();
        m_step = AutonStep.kStep1;
    }

    //this method will be called many times a second during the teleop period
    public void periodic(){
        if (m_path == AutonPath.kLeftWall){

        }
        else if (m_path == AutonPath.kRightWall){

        }
        else if (m_path == AutonPath.kRightLine){

        }
    }
}