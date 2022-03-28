package frc.robot;
//Xbox controller import
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//Drivetrain import
import frc.robot.Drivetrain.Gear;

public class PilotController {
    // declares limelight object for aiming launcher and targeting
    private LimelightVision m_limelightVision;

    //Declares controller, drivetrain, and shuffleboard objects used for moving the robot
    private XboxController m_controller;
    private Drivetrain m_drivetrain;
    private RobotShuffleboard m_shuffleboard;
    private Launcher m_launcher; 
    private Climber m_climber;

    //Declares velocity and turn scalars used to control input value for arcade drive and store shuffleboard values
    private double m_currentVelocityScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;
    private double m_currentTurnScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second (Constant is untested)
    SlewRateLimiter triggerFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_RATE_OF_CHANGE);
    SlewRateLimiter stickFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_TURN_RATE_OF_CHANGE);

    // Sysout counter
    int m_sysOutCounter = 0;

    //Boolean for Sysout counter
    boolean m_doSysOut = true;

    /**
     * Constuctor for the pilot controller
     */
    public PilotController(Drivetrain drivetrain, LimelightVision limelightVision, RobotShuffleboard shuffleboard, Launcher launcher, Climber climber){
        m_drivetrain = drivetrain;
        m_limelightVision = limelightVision;
        m_shuffleboard = shuffleboard;
        m_launcher = launcher;
        m_climber = climber;

        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
        //puts input scalar widgets on the shuffleboard
    }

    /**
     * Initialization method for the pilot controller
     * Calls init for drivetrain
     */
    public void init(){
        m_drivetrain.init();
    }
    
    /**
     * Periodic method for the pilot controller
     * Calls turnToTarget, arcadeDriveCmd, and controlGear to set up the buttons needed for targeting, switching gears, and controlling the drive train on the xbox pilot controller
     */
    public void periodic() {
        // When Start button is pressed we turn to target
        turnToTarget();
        // Calls the drivetrain to be utilized. Right trigger is forward, left trigger is backward, and left stick is turn
        arcadeDriveCmd();
        // Controls the gear with x button being high gear and y button being low gear 
        controlGear();
        // when left or right bumper are pressed turn the turret
        manualTurretCmd();

        //climberCmd();
     
        // Periodically updates encoder ticks to our actual current encoder position
        // double currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
        // double currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        /**
         * prints out our current right and left encoder ticks, prints only every 50 cycles 
         */
        // if ((++m_sysOutCounter % 50) == 0){
        //     System.out.println("Right Encoder Ticks: " + currentRightEncoderTicks);
        //     System.out.println("Left Encoder Ticks: " + currentLeftEncoderTicks);
        // }
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
        // When A is pressed we turn to target
        turnToTarget();
        // Calls the drivetrain to be utilized. Right trigger is forward, left trigger is backward, and left stick is turn
        arcadeDriveCmd();
        // Controls the gear with x button being high gear and y button being low gear 
        controlGear();
        // if(m_controller.getLeftBumper()){
        //     m_launcher.setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        // }
        // else if(m_controller.getRightBumper()){
        //     m_launcher.setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        // }
        // else{
        //     m_launcher.setTurretSpeed(0);
        // }

        // Periodically updates encoder ticks to our actual current encoder position
        // double currentLeftEncoderTicks = m_drivetrain.getLeftDriveEncoderPosition();
        // double currentRightEncoderTicks = m_drivetrain.getRightDriveEncoderPosition();
        /**
         * prints out our current right and left encoder ticks, prints only every 50 cycles 
         */
        if ((++m_sysOutCounter % 10) == 0){
            //System.out.println("Gyro: " + m_drivetrain.getGyro());
            //System.out.println("Distance to Target" + m_limelightVision.distToTarget());
        }


    }

    /**
     * Takes away the deadband value from any input (creates a deadband close to 0)
     * @param stickInput value that you want to pass through the deadband, likely only used for the left stick on the pilot controller
     * @return adjusted stick value
     */
    private double adjustForDeadband(double stickInput){
        double absoluteStickInput = Math.abs(stickInput);
        //if value is within deadband, return 0
        if(absoluteStickInput < RobotMap.PilotControllerConstants.STICK_DEADBAND) {
            return 0; 
        }
        //if value is greater than deadband, subtract deadband and reapply sign (forwards and backwards)
        else {
            //subtracts deadband so that there is not a jump in values
            absoluteStickInput -= RobotMap.PilotControllerConstants.STICK_DEADBAND;

            //assigning negative sign to negative inputs
            stickInput = Math.copySign(absoluteStickInput, stickInput);

            //adjusts for the limited range
            return stickInput / (1.0-RobotMap.PilotControllerConstants.STICK_DEADBAND);
        }
    }

    /**
     * Method to set our drivetrain motors to arcade drive controls. (Right trigger is forwards, left trigger is backwards, left stick is turn)
     */
    private void arcadeDriveCmd(){
        // triggerInput is for the velocity input forward and backwards
        double triggerInput = ((m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis()) * m_currentVelocityScalar);
        //double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        // leftStickXInput is for our current turn input
        //double leftStickXInput = stickFilter.calculate(m_controller.getLeftX());
        double leftStickXInput = (m_controller.getLeftX() * m_currentTurnScalar);

        // applies deadband method 
        leftStickXInput = adjustForDeadband(leftStickXInput);

        // limits the slew rate for trigger input
        triggerInput = triggerFilter.calculate(triggerInput);
        // limits the slew rate for left stick x input
        leftStickXInput = stickFilter.calculate(leftStickXInput);

        // passes in our variables from this method (calculations) into our arcade drive in drivetrain
        m_drivetrain.periodic(triggerInput, leftStickXInput);
    }

    /**
     * Method for changing the gear between high (X button) and low (Y button) (High for speed, low for torque)
     */
    private void controlGear(){
        // When x botton is pressed, drivetrain is switched into high gear, and when Y button is pressed drivetrain is switched into low gear
        if (m_controller.getXButtonPressed()){
            m_drivetrain.shiftGear(Gear.kHighGear);
            // Sets currentVelocityScalar equal to the value in the shuffleboard for the scalar
            m_currentVelocityScalar = m_shuffleboard.getHighVelocityScalar();
            m_currentTurnScalar = m_shuffleboard.getHighTurnScalar();

        } else if (m_controller.getYButtonPressed()){
            m_drivetrain.shiftGear(Gear.kLowGear);
            // Sets currentVelocityScalar equal to the value in the shuffleboard for the scalar
            m_currentVelocityScalar = m_shuffleboard.getLowVelocityScalar();
            m_currentTurnScalar = m_shuffleboard.getLowTurnScalar();
        }
    }

    /**
     * Turns to target when A button is pressed
     * TODO: Remove this method once we know turret autotargeting works because this method will be redundant 
     */
    private void turnToTarget(){
        // checks if Start button is pressed and executes code if it is
        if(m_controller.getStartButton()){
            m_limelightVision.enableLEDs();
            // change into low gear for defense and more accurate aim
            m_drivetrain.shiftGear(Gear.kLowGear);
            // checks if any part of the target is visible
            if (m_limelightVision.seeTarget() == true){
                // if target is outside of acceptable offset values, robot moves to aim at the target
                //TODO: fix so turret aims
                if (m_limelightVision.xAngleToTarget() < RobotMap.TOLERATED_TARGET_ERROR && m_limelightVision.xAngleToTarget() > -RobotMap.TOLERATED_TARGET_ERROR && m_limelightVision.yAngleToTarget() < RobotMap.TOLERATED_TARGET_ERROR && m_limelightVision.yAngleToTarget() > -RobotMap.TOLERATED_TARGET_ERROR){
                    m_drivetrain.periodic(0, 0);
                    // prints to let drivers know we are On Target
                    System.out.print("On Target");
                    return;  
                }
                // if target is within acceptable offset range, the robot stops moving
                else{
                    m_drivetrain.periodic(0, 0.5);
                }
            } 
            // if any part of the target is not visible, spin right until target is visible
            else if(m_limelightVision.seeTarget() == false){
                m_drivetrain.periodic(0, RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED);
            }
        }
        else {
            //m_limelightVision.disableLEDs();
        }
    }

    private void manualTurretCmd(){
        if(m_controller.getRightBumper()){
            m_launcher.setTurretSpeed(RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        else if(m_controller.getLeftBumper()){
            m_launcher.setTurretSpeed(-RobotMap.LauncherConstants.TURRET_ROTATION_SPEED);
        }
        else{
            m_launcher.setTurretSpeed(0);
        }
    }

    //Commented out because the climb controls were moved to the copilot controller, but we still have this here just in case
    // private void climberCmd(){
    //     if(m_controller.getAButton()){
    //         m_climber.climbCMD(RobotMap.ClimberConstants.CLIMBER_MOTOR_SPEED);
    //     }
    //     else{
    //         m_climber.climbCMD(0);
    //     }
        
    //     if(m_controller.getBButton()){
    //         m_climber.winchCMD(RobotMap.ClimberConstants.WINCH_MOTOR_SPEED);
    //     }
    //     else{
    //         m_climber.winchCMD(0);
    //     }
    // }
}
