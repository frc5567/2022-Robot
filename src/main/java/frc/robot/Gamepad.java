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
		//Port values for the different gamepad buttons (2021 had ports 4-10 used)
		Move_Climber_Up(4),
        Move_Climber_Down(5),
		Winch_Forward(6),
		Rev_Launcher(7),
		Idle_Climber_And_Winch(8);

		//Variable used to get port values 
		public final int portNum;

		//Enum constructor that allows for the different port values to be called upon
		GamePadControls(int newPortNum) {
			this.portNum = newPortNum;
		}
	}

	//Method to check if the Move Climber Up button was pressed
	public boolean getMoveClimberUp() {
		return super.getRawButton(GamePadControls.Move_Climber_Up.portNum);
	}
	
	//Method to check if the Move Climber Down button was pressed
	public boolean getMoveClimberDown() {
		return super.getRawButton(GamePadControls.Move_Climber_Down.portNum);
	}
    
    //Method to check if the Winch Forward button was pressed
	public boolean getWinchForward() {
		return super.getRawButton(GamePadControls.Winch_Forward.portNum);
	}

	//Method to check if the Rev Launcher button was pressed
	public boolean getRevLauncherPressed() {
		return super.getRawButtonPressed(GamePadControls.Rev_Launcher.portNum);
	}

	//Method to check if the Rev Launcher button was released from being held down
	public boolean getRevLauncherReleased() {
		return super.getRawButtonReleased(GamePadControls.Rev_Launcher.portNum);
	}

    //Method to check if the Idle Climber and Winch button was pressed
	public boolean getIdleClimberAndWinch() {
		return super.getRawButton(GamePadControls.Idle_Climber_And_Winch.portNum);
	}

	/**
	 * These must be extended because GenericHID is abstract
	 * We cannot delete these, nor make them private
	 */

	//Method that returns the x axis value of the joystick
	public double getX(Hand hand) {
		return getRawAxis(0);
	}

	//Method that returns the y axis value of the joystick
	public double getY(Hand hand) {
		return getRawAxis(1);
	}

}