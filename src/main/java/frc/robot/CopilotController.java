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
        m_gamePad = new GamePad(RobotMap.CopilotControllerConstants.GAMEPAD_PORT);
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
        //TODO: launcher controls, intake controls
    }

    

    /**
     * This method controlls the climber using three buttons to pass in values of power to two motors; the climber motor and the winch motor.
     */
    public void controlClimber(){
        //controls Climber motor with two buttons, up or down
        if(m_gamePad.getMoveClimberUp()){
            m_climber.climbMotor(RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else if(m_gamePad.getMoveClimberDown()){
            m_climber.climbMotor(RobotMap.ClimberConstants.CLIMBER_MOTOR_REVERSE_SPEED);
        }
        else{
            m_climber.climbMotor(0);
        }
        //controls Climber winch with one button, up
        if(m_gamePad.getMoveRobotUp()){
            m_climber.climbWinch(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
        }
        else{
            m_climber.climbWinch(0);
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