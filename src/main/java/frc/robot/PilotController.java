package frc.robot;
//Xbox controller import
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//Drivetrain import
import frc.robot.Drivetrain.Gear;

public class PilotController {
    // declares limelight object for aiming launcher and targeting
    private LimelightVision m_limelightVision;

    //Declares controller, drivetrain, shuffleboard, and launcher objects used for moving the robot and manually controlling the turret
    private XboxController m_controller;
    private Drivetrain m_drivetrain;
    private RobotShuffleboard m_shuffleboard;
    private Launcher m_launcher; 

    //Declares velocity and turn scalers used to limit input value for arcade drive and store shuffleboard values
    private double m_currentVelocityscaler = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER;
    private double m_currentTurnscaler = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALER;

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 3 units per second to prevent brownouts when accelerating too quickly
    SlewRateLimiter triggerFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_RATE_OF_CHANGE);
    SlewRateLimiter stickFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_TURN_RATE_OF_CHANGE);

    // Tracks how many cycles it has been since the last set of print outs to know when we can print again
    int m_sysOutCounter = 0;

    //Boolean for Sysout counter. When true we can print, when true we cannot
    boolean m_doSysOut = true;

    //Boolean for determining if the back button is being pressed
    boolean m_movingToClimb;

    /**
     * Constuctor for the pilot controller
     */
    public PilotController(Drivetrain drivetrain, LimelightVision limelightVision, RobotShuffleboard shuffleboard, Launcher launcher, Climber climber){
        //Sets the previously declared objects to be the same as the ones instantiated in Robot
        m_drivetrain = drivetrain;
        m_limelightVision = limelightVision;
        m_shuffleboard = shuffleboard;
        m_launcher = launcher;

        //Instantiates an xbox controller to take button/stick inputs
        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
    }

    /**
     * Initialization method for the pilot controller
     * Calls init for drivetrain because out of our two controller classes (Pilot Controller, Copilot Controller) this is the one that primarily controls the drivetrain
     */
    public void init(){
        m_drivetrain.init();
    }
    
    /**
     * Periodic method for the pilot controller
     * Calls turnToTarget as a backup in case turret targeting fails, arcadeDriveCmd to control the drivetrain, controlGear to switch between high and low gear, and manual turret command as a backup in case automatic targeting fails.
     */
    public void periodic() {
        // When Start button is pressed, we turn the drivetrain to center on a target
        turnToTarget();
        // Calls the drivetrain to be utilized. Right trigger is forward, left trigger is backward, and left stick is turn
        arcadeDriveCmd();
        // Controls the gear with x button being high gear and y button being low gear 
        controlGear();
        // when left or right bumper are pressed, turn the turret those directions respectively
        manualTurretCmd();

        if ((++m_sysOutCounter % 10) == 0){
            //System.out.println("Gyro: " + m_drivetrain.getGyro());
            // System.out.println("Distance to Target" + m_limelightVision.distToTarget());
            // System.out.println("Ty " + m_limelightVision.yAngleToTarget() + "  Tx " + m_limelightVision.xAngleToTarget() + "  Ta " + m_limelightVision.Ta());
        }
    }

    /**
     * Periodic method for pilot controller that includes manual testing controls
     */
    public void testPeriodic(){
        // When Start button is pressed, we turn the drivetrain to center on a target
        turnToTarget();
        // Calls the drivetrain to be utilized. Right trigger is forward, left trigger is backward, and left stick is turn
        arcadeDriveCmd();
        // Controls the gear with x button being high gear and y button being low gear 
        controlGear();
        // when left or right bumper are pressed, turn the turret those directions respectively
        manualTurretCmd();

        crawlCmd();

        if ((++m_sysOutCounter % 10) == 0){
            //System.out.println("Gyro: " + m_drivetrain.getGyro());
            //System.out.println("Distance to Target" + m_limelightVision.distToTarget());
            //System.out.println("Ty " + m_limelightVision.yAngleToTarget() + "  Tx " + m_limelightVision.xAngleToTarget() + "  Ta " + m_limelightVision.tAreaOfScreen());
        }
    }

    /**
     * Takes in an input from a joystick and creates a zone of values close to zero that are then turned to zero in order to prevent stick drift
     * @param stickInput value that you want to pass through the deadband, likely only used for the left stick on the pilot controller
     * @return adjusted input value
     */
    private double adjustForDeadband(double stickInput){
        //If the stick input is negative, change it to positive for calculation
        double absoluteStickInput = Math.abs(stickInput);
        //if value is within deadband (between 0 and 0.09), return 0
        if(absoluteStickInput < RobotMap.PilotControllerConstants.STICK_DEADBAND) {
            return 0; 
        }
        //if value is greater than deadband, subtract deadband and reapply sign to maintain the input direction (negative or positive)
        else {
            //subtracts deadband so that there is not a jump in input values
            absoluteStickInput -= RobotMap.PilotControllerConstants.STICK_DEADBAND;

            //assigns negative sign to negative inputs
            stickInput = Math.copySign(absoluteStickInput, stickInput);

            //Changes the ratio of inputs to match the smaller range of values when the deadband is included
            return stickInput / (1.0-RobotMap.PilotControllerConstants.STICK_DEADBAND);
        }
    }

    /**
     * Method to set our drivetrain motors to arcade drive controls. (Right trigger is forwards, left trigger is backwards, left stick is turn)
     */
    private void arcadeDriveCmd(){
        // This if statement makes sure that the trigger and stick input is only used when the back button is not being pressed (this gets rid of any zeroing issues)
        if(m_movingToClimb){
            m_drivetrain.shiftGear(Gear.kLowGear);
            m_drivetrain.periodic(RobotMap.PilotControllerConstants.CLIMB_CRAWL_SPEED, 0);
        }
        else{
            // Sets the triggerInput variable equal to a number between 0 and 1 based on inputs from bother triggers and scales those inputs to the scaler that we enter on the shuffleboard
            double triggerInput = ((m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis()) * m_currentVelocityscaler);
            //Sets the leftStickXInput variable equal to a number between 0 and 1 based on inputs from the x-axis of the left joystick and scales those inputs to the scaler we enter on the shuffleboard
            double leftStickXInput = (m_controller.getLeftX() * m_currentTurnscaler);

            // applies deadband method to the input from the left stick
            leftStickXInput = adjustForDeadband(leftStickXInput);

            // limits the rate of change on the trigger input by 3 units per second to prevent excessive acceleration and brownouts
            triggerInput = triggerFilter.calculate(triggerInput);
            // imits the rate of change on the left stick input by 3 units per second to prevent excessive acceleration and brownouts
            leftStickXInput = stickFilter.calculate(leftStickXInput);

            // passes our variables from this method's calculations into the drivetrain
            m_drivetrain.periodic(triggerInput, leftStickXInput);            
        }
    }

    /**
     * Method for changing the gear between high (X button) and low (Y button) (High for speed, low for torque)
     */
    private void controlGear(){
        // When x botton is pressed, drivetrain is switched into high gear, and when Y button is pressed drivetrain is switched into low gear
        if (m_controller.getXButtonPressed()){
            m_drivetrain.shiftGear(Gear.kHighGear);
            // Sets our the scalers on our drivetrain equal to the ones on the shuffleboard set for high gear
            m_currentVelocityscaler = m_shuffleboard.getHighVelocityScaler();
            m_currentTurnscaler = m_shuffleboard.getHighTurnScaler();

        } else if (m_controller.getYButtonPressed()){
            m_drivetrain.shiftGear(Gear.kLowGear);
            // Sets our the scalers on our drivetrain equal to the ones on the shuffleboard set for low gear
            m_currentVelocityscaler = m_shuffleboard.getLowVelocityScaler();
            m_currentTurnscaler = m_shuffleboard.getLowTurnScaler();
        }
    }

    /**
     * Turns the drivetrain to center on a limelight target when the start button is pressed
     */
    private void turnToTarget(){
        // Executes method while start button is being held
        if(m_controller.getStartButton()){
            double angleToTarget = m_limelightVision.xAngleToTarget();
            // Turns on the limelight for targeting
            m_limelightVision.enableLEDs();
            // change into low gear for defense and more accurate aim
            m_drivetrain.shiftGear(Gear.kLowGear);
            // checks if any part of the target is visible
            if (m_limelightVision.seeTarget() == true){
                // If the target is in the center of the screen, zero the drivetrain and print out a message to say that we are done targeting
                if (angleToTarget < RobotMap.TOLERATED_TARGET_ERROR && angleToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                    m_drivetrain.periodic(0, 0);
                    // prints to let drivers know we are On Target
                    System.out.print("On Target");
                    return;  
                }

                else{
                    //Assigns the negative or positive sign from our angleToTarget to our turn speed and turn to the target at that speed
                    double turnSpeed = Math.copySign(RobotMap.AutonConstants.TARGETING_SPEED, angleToTarget);
                    m_drivetrain.periodic(0, turnSpeed);
                }
            } 
        }
        else {
            // Turn off LEDs if start button is not being pressed
            m_limelightVision.disableLEDs();
        }
    }

    private void manualTurretCmd(){
        boolean turretTurning = false;
        // If the right bumper is being pressed, turn the turret right
        if(m_controller.getRightBumper()){
            turretTurning = true;
            m_launcher.setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        // If the left bumper is being pressed, turn the turret left
        else if(m_controller.getLeftBumper()){
            turretTurning = true;
            m_launcher.setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        // Zero the turret motor once if we are not pressing either button 
        else{
            if(turretTurning){
                m_launcher.setTurretSpeed(0);
                turretTurning = false;
            }
            
        }
    }

    /**
     * This method has the robot move slowly forward (0.2 speed in low gear) when the Back button on the pilot controller is being presssed.
     * It also sets the boolean m_movingToClimb as true if we are pressing the Back Button so we know not to use the stick input until it isn't being pressed.
     */
    private void crawlCmd(){
        if(m_controller.getBackButton()){
            // m_drivetrain.shiftGear(Gear.kLowGear);
            //m_drivetrain.periodic(RobotMap.PilotControllerConstants.CLIMB_CRAWL_SPEED, 0);
            m_movingToClimb = true;
        }
        else {
            m_movingToClimb = false;
        }
    }
}
