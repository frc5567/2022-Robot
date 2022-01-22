package frc.robot;

public class Auton{    
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

        kStep2,

        kStep3,

        kStep4,

        kStep5,

        kStep6,

        kStep7,

        kStop;
    }
    
    //declares instances of our drivetrain and the auton step enum
    private AutonStep m_step;
    private AutonPath m_path;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;

    /**
     * constructor for auton
     * @param drivetrain drivetrain mechanism on the robot
     */
    public Auton(Drivetrain drivetrain, Launcher launcher, Intake intake){
        m_drivetrain = drivetrain;
        m_launcher = launcher;
        m_intake = intake;
        m_step = AutonStep.kStep1;
        m_path = AutonPath.kLeftWall;
    }

    //this method will be run at the start of every auton period
    public void init(){
        m_drivetrain.zeroEncoders();
        m_launcher.zeroEncoders();
        m_step = AutonStep.kStep1;
    }

    //this method will be called many times a second during the auton period. currently all pseudo-code, need to create driveToTarget and turnToAngle methods 
    public void periodic(){
        if (m_path == AutonPath.kLeftWall){
            /*
            if(m_step == AutonStep.kStep1){

                if(driveToTarget(Speed, Distance)){
                    m_step = AutonStep.kStep2;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep2){
                
                if(turnToAngle(Clockwise Speed, target angle)){
                    m_step = AutonStep.kStep3;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep3){

                if(toggleIntakeExtension(m_intake.kExtended)){
                    m_step = AutonStep.kStep4;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep4){

                if(turn on intake)){
                    m_step = AutonStep.kStep5;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep5){

                if(driveToTarget(Speed, Distance)){
                    m_step = AutonStep.kStep6;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep6){

                if(turnToAngle(Clockwise Speed, 180)){
                    m_step = AutonStep.kStep7;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep7){

                if(target and launch both balls)){
                    m_step = AutonStep.kStop;
                }

                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStop){
                m_drivetrain.arcadeDrive(0, 0);
            }
            */

        }
        else if (m_path == AutonPath.kRightWall){
            /*
            -drive forward
            -turn slightly left
            -drive forward to ball
            -intake ball
            -turn about 180 degress back towards the hub
            -target and shoot both the pre-loaded ball and the picked-up ball
            */
        }
        else if (m_path == AutonPath.kRightLine){
            /*  
            -drive forward a much smaller amount
            -turn slightly right
            -drive forward to ball
            -intake ball
            -turn about 180 degress back towards the hub
            -target and shoot both the pre-loaded ball and the picked-up ball
            */
        }
    }
}