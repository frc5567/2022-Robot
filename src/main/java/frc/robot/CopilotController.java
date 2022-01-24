package frc.robot;

public class CopilotController {
    //declarations
    private GamePad m_gamePad;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private Climber m_climber;

    //constructor for copilot controller
    public CopilotController(Drivetrain drivetrain){
        m_gamePad = new GamePad(RobotMap.CopilotControllerConstants.XBOX_CONTROLLER_PORT);
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