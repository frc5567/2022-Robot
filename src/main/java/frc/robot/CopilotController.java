package frc.robot;

import frc.robot.Intake.IntakeState;
import frc.robot.Launcher.TrajectoryPosition;
import frc.robot.RobotMap.CopilotControllerConstants;
import edu.wpi.first.wpilibj.XboxController;

public class CopilotController {
    //declare objects for the different systems controlled by the coppilot controller
    private GamePad m_gamePad;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelight;
    //declares shuffleboard to be used for flywheel velocity testing
    private RobotShuffleboard m_shuffleboard;
    private double m_currentFlywheelVelocity = RobotMap.ShuffleboardConstants.FLYWHEEL_DEFAULT_VELOCITY;
    private double m_currentLaunchPreset = RobotMap.ShuffleboardConstants.DEFAULT_LAUNCH_PRESET;
    //for testing until we have a real gamepad
    private XboxController m_controller;

    boolean m_carwashRunning = false;


    /**
     * constructor for copilot controller- passes in all of the systems that we interact with in this class.
     * @param intake we pass in intake so the copilot can control the intake system
     * @param launcher we pass in launcher so the copilot can control the launcher system
     * @param shuffleboard we pass in shuffleboard so we can use flywheel velocity to pass in to targetAndLaunch
     * @param limelight we pass in limelight to enable and disable LEDs.
     */
    public CopilotController(Intake intake, Launcher launcher, RobotShuffleboard shuffleboard, LimelightVision limelight){
        //instatiates objects for copilot class
        m_intake = intake;
        m_launcher = launcher;
        m_shuffleboard = shuffleboard;
        m_limelight = limelight;
        
        m_gamePad = new GamePad(RobotMap.GamePadConstants.GAMEPAD_PORT);
        m_controller = new XboxController(RobotMap.CopilotControllerConstants.COPILOT_CONTROLLER_PORT);

    }

    /**
     * Initialization method for CopilotController class, should be called in robotInit
     */
    public void init(){
        m_intake.init();
        m_launcher.init();
        m_gamePad.init();
        //sets our flywheel velocity for testing to the value put into the shuffleboard
        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
        m_currentLaunchPreset = m_shuffleboard.getLaunchPreset();
        
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodic(){
        controlIntake();
        controlLauncher();

        m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
        m_currentLaunchPreset = m_shuffleboard.getLaunchPreset();
        m_shuffleboard.periodic();
    }

    /**
     * Periodic method for copilot controller that includes manual testing controls
     */
    public void testPeriodic(){
        // manualLauncherCmd();
        // manualIntakeCmd();

        // m_currentFlywheelVelocity = m_shuffleboard.getFlywheelVelocity();
        // m_currentLaunchPreset = m_shuffleboard.getLaunchPreset();
        // m_shuffleboard.periodic();

        // m_intake.indexing();

        // boolean sensor1 = m_intake.getMagazineSensor1();
        // boolean sensor2 = m_intake.getMagazineSensor2();

        // System.out.println("Current Distance: " + m_limelight.distToTarget(RobotMap.LimelightConstants.CAMERA_DEGREES_FROM_GROUND) + " ---- ");
        // System.out.print("Current RPM: "+ m_launcher.getRealSpeed());
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
            m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
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
        if (m_gamePad.getTargetAndLaunch()){
            m_limelight.enableLEDs();
            m_launcher.targetAndLaunch(m_shuffleboard.getFlywheelVelocity(), m_shuffleboard.getTargetFlywheelSpeed());
            m_intake.setMagazineSpeed(0);
        }
        //calls button in gamepad to spin flywheel up
        else if(m_gamePad.getManualLaunch()){
            m_launcher.launch();
        }
        //calls button in gamepad to burp
        else if(m_gamePad.getExpell()){
            m_launcher.expel();
        }
        //calls button in gamepad to to run carwash motor manually
        else if (m_gamePad.getCarwash()){
            m_carwashRunning = true;
            m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
        }
        //checks for feed command button, if it is pressed then don't zero it
        else if(m_gamePad.getFeedCMD()){
            return;
        }
        //zeros magazine only once
        else{
            if(m_carwashRunning){
                m_intake.setMagazineSpeed(0);
                m_carwashRunning = false;
            }
            m_launcher.setFlywheelSpeed(0);
            m_launcher.setFeederSpeed(0);
            m_launcher.setTurretSpeed(0);
            m_limelight.disableLEDs();
            m_intake.indexing();
            m_launcher.zeroFlywheelRevCounter();
        }

        //calls feeder command button to set feeder speed
        if(m_gamePad.getFeedCMD()){
            m_launcher.setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }
        //checks if target and launch is being pressed so we don't zero the feeder wheel
        else if(m_gamePad.getTargetAndLaunch() || m_gamePad.getManualLaunch()){
            return;
        }
        else{
            m_launcher.setFeederSpeed(0);
        }

        if (m_gamePad.getTrajectoryUpPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kUp);
        }
        else if (m_gamePad.getTrajectoryDownPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kDown);
        }
    }

    /**
     * Manually controls the launcher and the turret for testing
     */
    private void manualLauncherCmd(){
        //revs flywheel when A button is pressed
        if(m_controller.getAButton()){
            m_launcher.setFlywheelSpeed(m_currentFlywheelVelocity);
        }
        else{
            m_launcher.setFlywheelSpeed(0);
        }
        //sets trajectory position to up if start button is pressed
        if(m_controller.getStartButtonPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kUp);
        }
        //sets trajectory position to down if the back button is pressed
        else if(m_controller.getBackButtonPressed()){
            m_launcher.setTrajectoryPosition(TrajectoryPosition.kDown);
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

        //if B button is pressed turn on limelight, target, and launch
        if(m_controller.getBButton()){
            //Added for testing
            //m_launcher.setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
            m_limelight.enableLEDs();
            m_launcher.targetAndLaunch(m_shuffleboard.getTargetFlywheelSpeed(), m_shuffleboard.getTargetFlywheelSpeed());
            m_intake.setMagazineSpeed(0);
        }
        else {
            m_launcher.setFeederSpeed(0);
            m_launcher.setTurretSpeed(0);
           // m_limelight.disableLEDs();
            m_launcher.zeroTurretPosition();

        }
    }

    /**
     * Manually controls the intake for testing
     */
    private void manualIntakeCmd(){
        //two if statements to determine intake position
        //if X button is pressed turn on the magazine
        if(m_controller.getXButton()){
            m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
            //Commented out for testing purposes
            //m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        else {
            m_intake.setMagazineSpeed(0);
        }
        
        //if Y button is pressed turn on the roller wheels
        if(m_controller.getYButton()){
            m_intake.setRollerSpeed(RobotMap.IntakeConstants.ROLLER_SPEED);
        }
        else {
            m_intake.setRollerSpeed(0);
        }

        //if left bumper is pressed extend the intake
        if(m_controller.getLeftBumperPressed()){
            m_intake.setIntakeExtension(IntakeState.kExtended);
        }
        //if right bumper is pressed retract the intake
        else if(m_controller.getRightBumperPressed()) {
            m_intake.setIntakeExtension(IntakeState.kRetracted);
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
}