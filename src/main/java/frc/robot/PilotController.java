package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivetrain.Gear;
import frc.robot.Intake.IntakeState;
import edu.wpi.first.wpilibj.GenericHID;

/**
 * Creating a PilotController following the framework below

Interface
init()
periodic()
arcadeDriveCmd() // helper method that calculates proper values and passes into drivetrain
//specify a deadband and normalize the range
 */


public class PilotController {
    private LimelightVision m_limelight;
    private XboxController m_controller;
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private RobotShuffleboard m_shuffleboard;
    private double m_currentVelocityScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;
    private double m_currentTurnScalar = RobotMap.ShuffleboardConstants.DRIVE_DEFAULT_INPUT_SCALAR;

    //constructor for Pilot Controller
    public PilotController(){
        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
        m_drivetrain = new Drivetrain();
        m_launcher = new Launcher();
        m_intake = new Intake();
        m_shuffleboard = new RobotShuffleboard();
        m_limelight = new LimelightVision();

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

    //current button setup is temporary before a copilot controller is completed
    private void intakeCmd(){
        //when right bumper is held, activate intake
        if (m_controller.getRightBumperPressed()){
            m_intake.takeIn();
        }
    }

    //current button setup is temporary before a copilot controller is completed
    private void magazineCmd(){
        //when start button is held, run magazine
        if (m_controller.getStartButtonPressed()){
            m_intake.runMagazine();
        }
    }

    //current button setup is temporary before a copilot controller is completed
    private void toggleIntakeExtension(){
        //when the left stick button is pressed down, retract the intake
        if (m_controller.getLeftStickButtonPressed()){
            m_intake.toggleIntakeExtension(IntakeState.kRetracted);
        }
        //when the right stick button is pressed down, retract the intake
        else if (m_controller.getRightStickButtonPressed()){
            m_intake.toggleIntakeExtension(IntakeState.kExtended);
        }
    }
    /**
     * Turns to target when left bumper button is pressed
     */
    private void turnToTarget(){
        // checks if left bumper button is pressed and executes code if it is
        if(m_controller.getLeftBumperPressed()){
            // change into low gear for defense and more accurate aim
            m_drivetrain.shiftGear(Gear.kLowGear);
            // checks if any part of the target is visible
            if (m_limelight.seeTarget() == true){
                // if target is outside of acceptable offset values, robot moves to aim at the target
                if (m_limelight.xAngleToTarget() > 0.5 || m_limelight.xAngleToTarget() < -0.5 || m_limelight.yAngleToTarget() > 0.5 || m_limelight.yAngleToTarget() < -0.5){
                    m_drivetrain.arcadeDrive(m_limelight.distanceAdjustToTargetSpeed(),m_limelight.turnAngleAdjustToTargetSpeed());
                }
                // if target is within acceptable offset range, the robot stops moving
                else{
                    m_drivetrain.arcadeDrive(0, 0);
                    return;
                }
            } 
            // if any part of the target is not visible, spin right until target is visible
            else if(m_limelight.seeTarget() == false){
                m_drivetrain.arcadeDrive(0, RobotMap.LimelightConstants.MINIMUM_SEEKING_TARGET_SPEED);
            }
            // prints to let drivers know we are On Target
            System.out.print("On Target");
        }
    }

    public void init(){
        m_drivetrain.init();
    }

    public void periodic() {
        turnToTarget();
        arcadeDriveCmd();
        controlGear();
        intakeCmd();
        magazineCmd();
        toggleIntakeExtension();
    }
}
