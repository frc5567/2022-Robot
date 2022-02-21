package frc.robot;

import frc.robot.Intake.IntakeState;
import frc.robot.Launcher.TrajectoryPosition;
import edu.wpi.first.wpilibj.XboxController;

public class CopilotController {
    //declare objects for the different systems controlled by the coppilot controller
    private GamePad m_gamePad;
    private Launcher m_launcher;
    private Intake m_intake;
    private Climber m_climber;
    //declares shuffleboard to be used for flywheel velocity testing
    private RobotShuffleboard m_shuffleboard;
    private double m_currentFlywheelVelocity = RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY;
    //for testing until we have a real gamepad
    private XboxController m_controller;
    
    /**
     * constructor for copilot controller- passes in all of the systems that we interact with in this class.
     * @param intake we pass in intake so the copilot can control the intake system
     * @param launcher we pass in launcher so the copilot can control the launcher system
     * @param climber we pass in climber so the copilot can control the climber system
     */
    public CopilotController(Intake intake, Launcher launcher, Climber climber, RobotShuffleboard shuffleboard){
        //instatiates objects for copilot class
        m_intake = intake;
        m_launcher = launcher;
        m_climber = climber;
        m_shuffleboard = shuffleboard;
        
        m_gamePad = new GamePad(RobotMap.CopilotControllerConstants.GAMEPAD_PORT);

        //configures shuffleboard for flywheel testing
        m_shuffleboard.drivetrainShuffleboardConfig();
    }

    /**
     * Initialization method for CopilotController class, should be called in robotInit
     */
    public void init(){
        m_intake.init();
        m_launcher.init();
        m_climber.init();
        m_gamePad.init();
        //sets our flywheel velocity for testing to the value put into the shuffleboard
        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodic(){
        controlIntake();
        controlLauncher();
        controlClimber();
        //for testing purposes
        manualLauncherCmd();
        m_shuffleboard.periodic();
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
        if (m_gamePad.getIntakeCMD()){
            // If the button getInakeCMD is pressed and the intake is extended, we activate the intake
            m_intake.takeIn();
        }
        else{
            // If the button is not pressed or the intake is not extended, intake and magazine motors don't run
            m_intake.setRollerSpeed(0);
            m_intake.setMagazineSpeed(0);
        }
    }

    /**
     * This method controls the launcher (and turret) with three buttons, one for automatically targeting and launching, and two for setting the position of the trajectory controller
     */
    private void controlLauncher(){
        //uses one button to aim and launch
        if (m_gamePad.getLaunchCMD()){
            m_launcher.targetAndLaunch();
        }
        else{
            m_launcher.setFlywheelSpeed(0);
            m_launcher.setTurretSpeed(0);
        }

        if (m_gamePad.getTrajectoryUpPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kUp);
        }
        else if (m_gamePad.getTrajectoryDownPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kDown);
        }
    }

    /**
     * This method controls the climber using three buttons to pass in values of power to two motors; the climber motor and the winch motor.
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
     * Manually controls the launcher and the turret for testing
     */
    private void manualLauncherCmd(){
        if(m_controller.getLeftBumper()){
            m_launcher.setTurretSpeed(RobotMap.LauncherConstants.NEGATIVE_TURRET_ROTATION_SPEED);
        }
        else if(m_controller.getRightBumper()){
            m_launcher.setTurretSpeed(RobotMap.LauncherConstants.POSITIVE_TURRET_ROTATION_SPEED);
        }
        else{
            m_launcher.setTurretSpeed(0);
        }

        if(m_controller.getAButton()){
            m_launcher.setFlywheelSpeed(m_currentFlywheelVelocity);
        }
        else{
            m_launcher.setFlywheelSpeed(0);
        }
    }

    /**
     * Manually controls the intake for testing 
     */
    private void manualIntakeCmd(){
        //two if statements to determine intake position
        if(m_controller.getXButton()){
            m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        else if(m_controller.getYButton()){
            m_intake.setIntakeExtension(IntakeState.kRetracted);
        }
        
        double triggerInput = (m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());
        m_intake.setRollerSpeed(triggerInput);
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