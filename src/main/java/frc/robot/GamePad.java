package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

public class GamePad extends GenericHID {
	/**
	 * Constructor, used for calling super constructor
	 * @param port Port the gamepad is connected to
	 */
	public GamePad(final int port) {
		super(port);
	}

	/**
	 * Initialization method for GamePad
	 */
	public void init(){
	}

	// this enum defines the buttons and what they do when active
	private enum GamePadControls {
		//Port values for the different gamepad buttons
		Lift_Up(4),
		Manual_Feed(3),
		Manual_Launch(2),
		Winch_Up(1),
		IntakeCMD(5),
		Retract_Intake(6),
		Extend_Intake(7),
		targetAndLaunchCMD(8),
		Carwash(9),
		ExpelCMD(10);
        
		//Variable used to get port values 
		public final int portNum;

		//Enum constructor that allows for the different port values to be called upon
		GamePadControls(int newPortNum) {
			this.portNum = newPortNum;
		}
	}

	/**
	 * Method to check if the Extend Intake button was pressed
	 */
	public boolean getExtendIntakePressed() {
		return super.getRawButton(GamePadControls.Extend_Intake.portNum);
	}

	/**
	 * Method to check if the Retract Intake button was pressed
     */
	public boolean getRetractIntakePressed() {
		return super.getRawButton(GamePadControls.Retract_Intake.portNum);
	}

	/**
	 * Method to check if the IntakeCMD button was pressed
	 */
	public boolean getIntakeCMD() {
		return super.getRawButton(GamePadControls.IntakeCMD.portNum);
	}

	/**
	 * Method to check if the Launch CMD button was pressed
	 */
	public boolean getTargetAndLaunch() {
		return super.getRawButton(GamePadControls.targetAndLaunchCMD.portNum);
	}

	public boolean getTargetAndLaunchPressed(){
		return super.getRawButtonPressed(GamePadControls.targetAndLaunchCMD.portNum);
	}

	/**
	 * Method to check if the Trajectory Up button was pressed
	 */
	public boolean getLiftButton() {
		return super.getRawButton(GamePadControls.Lift_Up.portNum);
	}

	/**
	 * Method to check if the Trajectory Down button was pressed
	 */
	public boolean getWinchButton() {
		return super.getRawButton(GamePadControls.Winch_Up.portNum);
	}

	/**
	 * Method to check if the Move Climber Up button was pressed
	 */
	public boolean getExpell() {
		return super.getRawButton(GamePadControls.ExpelCMD.portNum);
	}

	/**
	 * Method to check if the Move Climber Down button was pressed 
	 */
	public boolean getCarwash(){
		return super.getRawButton(GamePadControls.Carwash.portNum);
	}
		
	/**
	 * Method to check if the Move Robot Up button was pressed
	 */
	public boolean getFeedCMD() {
		return super.getRawButton(GamePadControls.Manual_Feed.portNum);
	}

	/**
	 * Method to check if the Manual Launch button was pressed
	 */
	public boolean getManualLaunch() {
		return super.getRawButton(GamePadControls.Manual_Launch.portNum);
	}
}
