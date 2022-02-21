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

	/**
	 * Actions each button performs
	 * TODO: Change numbers to correct port number
	 */

	// this enum defines the buttons and what they do when active
	private enum GamePadControls {
		//Port values for the different gamepad buttons
		Extend_Intake(0),
		Retract_Intake(1),
		IntakeCMD(2),
		LaunchCMD(3),
		Trajectory_Up(4),
		Trajectory_Down(5),
		Move_Climber_Up(6),
		Move_Climber_Down(7),
        Move_Robot_Up(8);

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
	public boolean getIntakeCMD() {
		return super.getRawButton(GamePadControls.IntakeCMD.portNum);
	}

	/**
	 * Method to check if the Launch CMD button was pressed
	 */
	public boolean getLaunchCMD() {
		return super.getRawButtonPressed(GamePadControls.LaunchCMD.portNum);
	}

	/**
	 * Method to check if the Trajectory Up button was pressed
	 */
	public boolean getTrajectoryUpPressed() {
		return super.getRawButtonPressed(GamePadControls.Trajectory_Up.portNum);
	}

	/**
	 * Method to check if the Trajectory Down button was pressed
	 */
	public boolean getTrajectoryDownPressed() {
		return super.getRawButtonPressed(GamePadControls.Trajectory_Down.portNum);
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
