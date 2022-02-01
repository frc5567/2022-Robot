package frc.robot;
//Xbox controller import
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//Drivetrain import
import frc.robot.Drivetrain.Gear;

public class PilotController {
    private LimelightVision m_limelight;

    //Declares controller, drivetrain, and shuffleboard objects
    private XboxController m_controller;
    private Drivetrain m_drivetrain;
    private RobotShuffleboard m_shuffleboard;

    //Declares scalars
    private double m_currentVelocityScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;
    private double m_currentTurnScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;

    // Creates a SlewRateLimiter that limits the rate of change of the signal to 0.5 units per second (Constant is untested)
    SlewRateLimiter leftTriggerFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_RATE_OF_CHANGE);
    SlewRateLimiter rightTriggerFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_RATE_OF_CHANGE);
    SlewRateLimiter stickFilter = new SlewRateLimiter(RobotMap.PilotControllerConstants.SLEW_SIGNAL_RATE_OF_CHANGE);

    /**
     * Constuctor for the pilot controller
     */
    public PilotController(){
        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
        m_drivetrain = new Drivetrain();
        m_shuffleboard = new RobotShuffleboard();
        m_limelight = new LimelightVision();

        //puts input scalar widgets on the shuffleboard
        m_shuffleboard.drivetrainShuffleboardConfig();
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
        double triggerInput = rightTriggerFilter.calculate(m_controller.getRightTriggerAxis()) - leftTriggerFilter.calculate(m_controller.getLeftTriggerAxis());
        //double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        // leftStickXInput is for our current turn input
        double leftStickXInput = stickFilter.calculate(m_controller.getLeftX());
        //double leftStickXInput = m_controller.getLeftX();

        // applies deadband method 
        leftStickXInput = adjustForDeadband(leftStickXInput);

        // Multiplies triggerInput and currentVelocityScalar and sets triggerInput equal to product
        triggerInput *= m_currentVelocityScalar;
        // Multiplies triggerInput and currentTurnScalar and sets triggerInput equal to the product.
        leftStickXInput *= m_currentTurnScalar;

        // passes in our variables from this method (calculations) into our arcade drive in drivetrain
        m_drivetrain.arcadeDrive(triggerInput, leftStickXInput);
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
     * Turns to target when left bumper button is pressed
     * TODO: Remove this method once we know turret autotargeting works because this method will be redundant 
     */
    private void turnToTarget(){
        // checks if left bumper button is pressed and executes code if it is
        if(m_controller.getLeftBumperPressed()){
            // change into low gear for defense and more accurate aim
            m_drivetrain.shiftGear(Gear.kLowGear);
            // checks if any part of the target is visible
            if (m_limelight.seeTarget() == true){
                // if target is outside of acceptable offset values, robot moves to aim at the target
                if (m_limelight.xAngleToTarget() < RobotMap.PilotControllerConstants.TOLERATED_TARGET_ERROR && m_limelight.xAngleToTarget() > -RobotMap.PilotControllerConstants.TOLERATED_TARGET_ERROR && m_limelight.yAngleToTarget() < RobotMap.PilotControllerConstants.TOLERATED_TARGET_ERROR && m_limelight.yAngleToTarget() > -RobotMap.PilotControllerConstants.TOLERATED_TARGET_ERROR){
                    m_drivetrain.arcadeDrive(0, 0);
                    // prints to let drivers know we are On Target
                    System.out.print("On Target");
                    return;  
                }
                // if target is within acceptable offset range, the robot stops moving
                else{
                    m_drivetrain.arcadeDrive(m_limelight.distanceAdjustToTargetSpeed(),m_limelight.turnAngleAdjustToTargetSpeed());
                }
            } 
            // if any part of the target is not visible, spin right until target is visible
            else if(m_limelight.seeTarget() == false){
                m_drivetrain.arcadeDrive(0, RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED);
            }
            
        }
    }

    /**
     * Initialization method for the pilot controller
     */
    public void init(){
        m_drivetrain.init();

    }
    /**
     * Periodic method for the pilot controller
     */
    public void periodic() {
        turnToTarget();
        arcadeDriveCmd();
        controlGear();
    }
}
