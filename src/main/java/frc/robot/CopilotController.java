package frc.robot;

import frc.robot.Intake.IntakeState;

public class CopilotController {
    //declarations
    private GamePad m_gamePad;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private Climber m_climber;

    /**
     * constructor for copilot controller- instantiates all of the systems that we interact with in this class.
     */
    public CopilotController(){
        m_gamePad = new GamePad(RobotMap.CopilotControllerConstants.GAMEPAD_PORT);
        m_intake = new Intake();
        m_launcher = new Launcher();
        //pass in turret here?
        m_climber = new Climber();
        m_drivetrain = new Drivetrain();
    }

    /**
     * this method should be called once in robot
     */
    public void initCopilot(){
        m_drivetrain.init();
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodicCopilot(){
        controlClimber();
        controlIntake();
        controlLauncher();
    }

    /**
     * This method controlls the intake using three buttons
     */
    private void controlIntake(){
        //two if statements to determine intake position
        if(m_gamePad.getExtendIntakePressed()){
            m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        else if(m_gamePad.getRetractIntakePressed()){
            m_intake.setIntakeExtension(IntakeState.kRetracted);
        }
        //if statement to control the power of the intake
        if (m_gamePad.getIntakeCMDPressed()){
            m_intake.takeIn();
        }
        else{
            m_intake.setFrontRollerSpeed(0);
            m_intake.setMagazineSpeed(0);
        }
    }

    /**
     * This method controls the launcher (and turret) with two buttons, one for revving and one for advancing balls into the launcher
     */
    private void controlLauncher(){
        //uses one button to aim and rev
        if (m_gamePad.getRevPressed()){
            m_launcher.prepareLaunch();
        }
        //uses one button to advance the feeder wheel
        if (m_gamePad.getLaunchCMD()){
            m_launcher.launch();
        }
    }

    /**
     * This method controlls the climber using three buttons to pass in values of power to two motors; the climber motor and the winch motor.
     */
    private void controlClimber(){
        //controls Climber motor with two buttons, up or down
        if(m_gamePad.getMoveClimberUp()){
            m_climber.climbCMD(RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else if(m_gamePad.getMoveClimberDown()){
            m_climber.climbCMD(RobotMap.ClimberConstants.CLIMBER_MOTOR_REVERSE_SPEED);
        }
        else{
            m_climber.climbCMD(0);
        }
        //controls Climber winch with one button, up
        if(m_gamePad.getMoveRobotUp()){
            m_climber.WinchCMD(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
        }
        else{
            m_climber.WinchCMD(0);
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