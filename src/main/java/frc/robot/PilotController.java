package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivetrain.Gear;
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
    private XboxController m_controller;
    private Drivetrain m_drivetrain;

    //constructor for Pilot Controller
    public PilotController(){
        m_controller = new XboxController(RobotMap.PilotControllerConstants.XBOX_CONTROLLER_PORT);
        m_drivetrain = new Drivetrain();
    }

    // this method takes away the deadband value from any input (creates a deadband close to 0)
    private double adjustForDeadband(double stickInput){
        double absoluteStickInput = Math.abs(stickInput);
        //if value is within deadband, return 0
        if(absoluteStickInput < RobotMap.PilotControllerConstants.PILOT_CONTROLLER_STICK_DEADBAND) {
            return 0; 
        }
        //if value is greater than deadband, subtract deadband and reapply sign (forwards and backwards)
        else {
            //subtracts deadband so that there is not a jump in values
            absoluteStickInput -= RobotMap.PilotControllerConstants.PILOT_CONTROLLER_STICK_DEADBAND;

            //assigning negative sign to negative inputs
            stickInput = Math.copySign(absoluteStickInput, stickInput);

            //adjusts for the limited range
            return stickInput / (1.0-RobotMap.PilotControllerConstants.PILOT_CONTROLLER_STICK_DEADBAND);
        }
    }

    private void arcadeDriveCmd(){
        double triggerInput = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
        double leftStickXInput = m_controller.getLeftX();

        // applies deadband method 
        leftStickXInput = adjustForDeadband(leftStickXInput);
        // passes in our variables from this method (calculations) into our arcade drive in drivetrain
        m_drivetrain.arcadeDrive(triggerInput, leftStickXInput);
    }

    private void controlGear(){
        // When x botton is pressed, drivetrain is switched into high gear
        if (m_controller.getXButtonPressed()){
            m_drivetrain.shiftGear(Gear.kHighGear);

        } else if (m_controller.getYButtonPressed()){
            m_drivetrain.shiftGear(Gear.kLowGear);
        }
    }

    public void init(){
        m_drivetrain.init();
    }

    public void periodic() {
        arcadeDriveCmd();
        controlGear();
    }



}
