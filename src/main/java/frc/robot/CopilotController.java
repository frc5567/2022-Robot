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
    private double m_currentLaunchPreset = RobotMap.ShuffleboardConstants.DEFAULT_LAUNCH_PRESET;
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
        
        m_controller = new XboxController(RobotMap.CopilotControllerConstants.COPILOT_CONTROLLER_PORT);
        //m_gamePad = new GamePad(RobotMap.CopilotControllerConstants.GAMEPAD_PORT);

    }

    /**
     * Initialization method for CopilotController class, should be called in robotInit
     */
    public void init(){
        m_intake.init();
        m_launcher.init();
        m_climber.init();
        //m_gamePad.init();
        //sets our flywheel velocity for testing to the value put into the shuffleboard
        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
        m_currentLaunchPreset = m_shuffleboard.getLaunchPreset();
        
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodic(){
        //when testing with xbox controller, comment out intake, launcher, and climber
        //TODO: uncomment these when testing is complete
        //controlIntake();
        //controlLauncher();
        //controlClimber();
        //for testing purposes
        manualLauncherCmd();
        manualIntakeCmd();
        manualClimberCmd();
        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
        m_currentLaunchPreset = m_shuffleboard.getLaunchPreset();
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
            // If the button is not pressed or the intake is not extended, set intake to not run
            m_intake.setRollerSpeed(0);
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
            m_climber.climbCMD(-RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else{
            m_climber.climbCMD(0);
        }
        //controls Climber winch with one button, up
        if(m_gamePad.getMoveRobotUp()){
            m_climber.winchCMD(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
        }
        else{
            m_climber.winchCMD(0);
        }
    }

    /**
     * Manually controls the launcher and the turret for testing
     */
    private void manualLauncherCmd(){
        if(m_controller.getAButton()){
            m_launcher.setFlywheelSpeed(m_currentFlywheelVelocity);
        }
        else{
            m_launcher.setFlywheelSpeed(0);
        }

        // if(m_controller.getBButton()){
        //     if(m_currentLaunchPreset == 0){
        //         m_launcher.lowPreset10();
        //     }
        //     if(m_currentLaunchPreset == 1){
        //         m_launcher.lowPreset20();
        //     }
        //     if(m_currentLaunchPreset == 2){
        //         m_launcher.highPreset10();
        //     }
        //     if(m_currentLaunchPreset == 3){
        //         m_launcher.highPreset20();
        //     }
        //     else{
        //         System.out.println(m_currentLaunchPreset + "is not a vald preset");
        //         return;
        //     }
        //}
        if(m_controller.getBButton()){
            //Added for testing
            //m_launcher.setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
            //Commented out for testing purposes
            m_launcher.targetAndLaunch();
        }
        //else {
            //Added for testing
            //m_launcher.setFeederSpeed(0);
       // }
    }

    /**
     * Manually controls the intake for testing
     */
    private void manualIntakeCmd(){
        //two if statements to determine intake position
        if(m_controller.getXButton()){
            m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
            //Commented out for testing purposes
            //m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        else {
            //Added for Testing
            m_intake.setMagazineSpeed(0);
        }
        
        if(m_controller.getYButton()){
            m_intake.setRollerSpeed(RobotMap.IntakeConstants.ROLLER_SPEED);
            //Commented out for testing purposes
            //m_intake.setIntakeExtension(IntakeState.kRetracted);
        }
        else {
            //Added for testing
            m_intake.setRollerSpeed(0);
        }

        if(m_controller.getLeftBumperPressed()){
            m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        else if(m_controller.getRightBumperPressed()) {
            m_intake.setIntakeExtension(IntakeState.kRetracted);
        }  
              
        //Commented out for testing purposes. Also review double triggerInput (may not need to subtract the two but instead get one triggerAxis)
        //double triggerInput = (m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis());
        //m_intake.setRollerSpeed(triggerInput);
    }

    private void manualClimberCmd(){
        if(m_controller.getLeftStickButton()){
            m_climber.climbCMD(RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else {
            m_climber.climbCMD(0);
        }

        if(m_controller.getRightStickButton()){
            m_climber.winchCMD(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
        }
        else {
            m_climber.winchCMD(0);
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