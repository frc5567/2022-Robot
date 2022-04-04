package frc.robot;

import frc.robot.Intake.IntakeState;


public class CopilotController {
    //declare objects for the different systems controlled by the coppilot controller
    private GamePad m_gamePad;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelight;
    private Climber m_climber;
    //declares shuffleboard to be used for flywheel velocity testing
    private RobotShuffleboard m_shuffleboard;

    boolean m_carwashRunning = false;

    // Tracks how many cycles it has been since the last set of print outs to know when we can print again
    int m_sysOutCounter = 0;


    /**
     * constructor for copilot controller- passes in all of the systems that we interact with in this class.
     * @param intake we pass in intake so the copilot can control the intake system
     * @param launcher we pass in launcher so the copilot can control the launcher system
     * @param shuffleboard we pass in shuffleboard so we can use flywheel velocity to pass in to targetAndLaunch
     * @param limelight we pass in limelight to enable and disable LEDs.
     */
    public CopilotController(Intake intake, Launcher launcher, RobotShuffleboard shuffleboard, LimelightVision limelight, Climber climber){
        //instatiates objects for copilot class
        m_intake = intake;
        m_launcher = launcher;
        m_shuffleboard = shuffleboard;
        m_limelight = limelight;
        m_climber = climber;

        m_gamePad = new GamePad(RobotMap.GamePadConstants.GAMEPAD_PORT);
    }

    /**
     * Initialization method for CopilotController class, should be called in robotInit
     */
    public void init(){
        m_intake.init();
        m_launcher.init();
        m_gamePad.init();
    }

    /**
     * This method should be called periodically in Teleop in order to control all systems
     */
    public void periodic(){
        m_shuffleboard.periodic();
        m_launcher.periodic();
        controlIntake();
        controlLauncher();
        controlClimber();

        if ((++m_sysOutCounter % 10) == 0){
            //put any sys.outs here
        }
    }

    /**
     * Periodic method for copilot controller that includes manual testing controls
     */
    public void testPeriodic(){
        System.out.println("First Sensor   [" + m_intake.getMagazineSensor0() + "] --- " + "Second Sensor    [" + m_intake.getMagazineSensor1() + "]");
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
     * This method controls the launcher (and turret) with three buttons, one for automatically targeting and launching,
     * and two for setting the position of the trajectory controller
     */
    private void controlLauncher(){
        //uses one button to aim and launch
        if (m_gamePad.getTargetAndLaunch()){
            m_limelight.enableLEDs();
            m_launcher.targetAndLaunch();
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
            //This is here in order to allow us to use carwash and other buttons at the same time. look to line 156 for carwashCMD control
        }
        //checks for feed command button, if it is pressed then don't zero it
        else if(m_gamePad.getFeedCMD()){
            //This is here in order to allow us to use feeder and other buttons at the same time. look to line 152 for feedCMD control
        }
        //zeros magazine only once
        else{
            m_launcher.setFlywheelSpeed(0);
            m_launcher.setFeederSpeed(0);
            m_launcher.zeroTurretPosition();
            m_limelight.disableLEDs();
            m_intake.indexing();
            m_launcher.resetSecondBallTracker();
        }

        //calls feeder command button to set feeder speed
        if(m_gamePad.getFeedCMD()){
            m_launcher.setFeederSpeed(RobotMap.LauncherConstants.FEEDING_SPEED);
        }

        if (m_gamePad.getCarwash() && !m_gamePad.getTargetAndLaunch()){
            m_carwashRunning = true;
            m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
        }
        else if(!m_gamePad.getTargetAndLaunch() && !m_gamePad.getExpell()){
            if(m_carwashRunning){
                // m_intake.setMagazineSpeed(0);
                m_carwashRunning = false;
            }
        }
    }
    /**
     * This method controls the climber with two buttons, one for lifting the climbing mechanism up, and another for retracting the winch
     */
    private void controlClimber(){
        //uses one button to lift the climbing mechanism up
        if (m_gamePad.getLiftButton()){
            m_climber.setLiftSpeed(RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
        }
        else{
            m_climber.setLiftSpeed(0);
        }
        //uses one button to retract the winch
        if (m_gamePad.getWinchButton()){
            m_climber.setWinchSpeed(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
        }
        else{
            m_climber.setWinchSpeed(0);
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