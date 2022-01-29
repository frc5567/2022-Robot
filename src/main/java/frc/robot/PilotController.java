package frc.robot;
//Xbox controller import
import edu.wpi.first.wpilibj.XboxController;
//Drivetrain import
import frc.robot.Drivetrain.Gear;

public class PilotController {
    //Declares controller, drivetrain, and shuffleboard objects
    private XboxController m_controller;
    private Drivetrain m_drivetrain;
    private RobotShuffleboard m_shuffleboard;

    //Declares scalars
    private double m_currentVelocityScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;
    private double m_currentTurnScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;

    /**
     * Constuctor for the pilot controller
     */
    public PilotController(){
        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
        m_drivetrain = new Drivetrain();
        m_shuffleboard = new RobotShuffleboard();

        //puts input scalar widgets on the shuffleboard
        m_shuffleboard.drivetrainShuffleboardConfig();
    }

    // this method takes away the deadband value from any input (creates a deadband close to 0)
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
        double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        // leftStickXInput is for our current turn input
        double leftStickXInput = m_controller.getLeftX();

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
     * Initialization method for the pilot controller
     */
    public void init(){
        m_drivetrain.init();

    }
    /**
     * Periodic method for the pilot controller
     */
    public void periodic() {
        arcadeDriveCmd();
        controlGear();
    }
}
