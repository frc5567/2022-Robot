package frc.robot;

//imports, still need vision update
import frc.robot.Drivetrain;
import frc.robot.Launcher;
import frc.robot.Intake;
import frc.robot.Climber;
import frc.robot.Gamepad;
import edu.wpi.first.wpilibj.GenericHID.Hand;

public class CopilotController {
    //declarations
    private Gamepad m_gamepad;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private Climber m_climber;

    //constructor for copilot controller
    public CopilotController(Drivetrain drivetrain){
        m_gamePad = new GamePad(RobotMap.GAMEPAD_PORT);
        m_intake = new Intake();
        m_launcher = new Launcher();
        //pass in turret here?
        m_climber = new Climber();
        m_drivetrain = drivetrain;
    }

    public void initCopilot(){
        m_drivetrain.init();
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodicCopilot(){
        controlClimber();
    }

    public void controlClimber(){
        if(m_gamePad.getMoveClimberUp()){
            //move up
        }
        else if(m_gamePad.getMoveClimberDown()){
            //move down
        }
    }
    /**
     * @return the launcher
     */
    public Launcher getLauncher() {
        return m_launcher;
    }
    /**
     * @return the intake
     */
    public Intake getIntake() {
        return m_intake;
    }
    /**
     * @return the climber
     */
    public Climber getClimber() {
        return m_climber;
    }
}