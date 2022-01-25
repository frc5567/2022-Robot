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
	 * Actions each button performs
	 * Change numbers to correct port number
	 */
	// this enum defines the buttons and what they do when active
	private enum GamePadControls {
		//Port values for the different gamepad buttons
		Extend_Intake(1),
		Retract_Intake(2),
		IntakeCMD(3),
		Aim_Launcher(4),
		Rev_Expell(5),
		Rev_Low(6),
		Rev_High(7),
		LaunchCMD(8),
		Move_Climber_Up(9),
		Move_Climber_Down(10),
        Move_Robot_Up(11);

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
		return super.getRawButtonPressed(GamePadControls.Extend_Intake.portNum);
	}

	/**
	 * Method to check if the Retract Intake button was pressed
	*/
	public boolean getRetractIntakePressed() {
		return super.getRawButtonPressed(GamePadControls.Retract_Intake.portNum);
	}

	/**
	 * Method to check if the IntakeCMD button was pressed
	*/
	public boolean getIntakeCMDPressed() {
		return super.getRawButtonPressed(GamePadControls.IntakeCMD.portNum);
	}

	/**
	 * Method to check if the IntakeCMD button was released 
	*/
	public boolean getIntakeCMDReleased(){
		return super.getRawButtonReleased(GamePadControls.IntakeCMD.portNum);
	}


	/**
	 * Method to check if the Aim Launcher button was pressed
	*/
	public boolean getAimLauncher() {
		return super.getRawButtonPressed(GamePadControls.Aim_Launcher.portNum);
	}

	/**
	 * Method to check if the Rev Expell button was pressed
	*/
	public boolean getRevExpellPressed() {
		return super.getRawButtonPressed(GamePadControls.Rev_Expell.portNum);
	}

	/**
	 * Method to check if the Rev Expell button was released from being held down
	 */
	public boolean getRevExpellReleased() {
		return super.getRawButtonReleased(GamePadControls.Rev_Expell.portNum);
	}


	/**
	 * Method to check if the Rev Low button was pressed
	*/
	public boolean getRevLowPressed() {
		return super.getRawButtonPressed(GamePadControls.Rev_Low.portNum);
	}

	/**
	 * Method to check if the Rev Low button was released from being held down
	*/
	public boolean getRevLowReleased() {
		return super.getRawButtonReleased(GamePadControls.Rev_Low.portNum);
	}


	/**
	 * Method to check if the Rev High button was pressed
	*/
	public boolean getRevHighPressed() {
		return super.getRawButtonPressed(GamePadControls.Rev_High.portNum);
	}

	/**
	 * Method to check if the Rev High button was released from being held down
	*/
	public boolean getRevHighReleased() {
		return super.getRawButtonReleased(GamePadControls.Rev_High.portNum);
	}


	/**
	 * Method to check if the Launch CMD button was pressed
	*/
	public boolean getLaunchCMD() {
		return super.getRawButtonPressed(GamePadControls.LaunchCMD.portNum);
	}

	/**
	 * Method to check if the Move Climber Up button was pressed
	*/
	public boolean getMoveClimberUp() {
		return super.getRawButton(GamePadControls.Move_Climber_Up.portNum);
	}

	/**
	 * Method to check if the Move Climber Down button was pressed 
	*/
	public boolean getMoveClimberDown(){
		return super.getRawButton(GamePadControls.Move_Climber_Down.portNum);
	}
		
	/**
	 * Method to check if the Move Robot Up button was pressed
	*/
	public boolean getMoveRobotUp() {
		return super.getRawButton(GamePadControls.Move_Robot_Up.portNum);
	}	
}