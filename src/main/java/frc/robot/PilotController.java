package frc.robot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Drivetrain.Gear;
import frc.robot.Launcher.LauncherState;
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
    private Launcher m_launcher;

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

    //This method is for testng the flywheel before the CoPilot Controller is ready
    private void manualLauncherCmd(){
        //when the a button is pressed, flywheel is revved to launch speed. when b button is pressed, flywheel returns to idle
        if (m_controller.getAButtonPressed()){
            m_launcher.setState(LauncherState.kLaunch);

        } else if (m_controller.getBButtonPressed()){
            m_launcher.setState(LauncherState.kIdle);
        }
    }

    public void init(){
        m_drivetrain.init();
    }

    public void periodic() {
        arcadeDriveCmd();
        controlGear();
        manualLauncherCmd();
    }



}
