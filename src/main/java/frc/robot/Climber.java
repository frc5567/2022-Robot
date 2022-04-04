package frc.robot;

// Import motor
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {
    //Declares lift motor to move the climb arm up, and declares winch motor to pull the robot up to the bar with the rope 
    private VictorSPX m_liftMotor;
    private VictorSPX m_winchMotor; 

    //Member variables used to make sure we aren't calling on the CAN bus constantly by repetedly setting the motors to zero 
    private double m_liftCurrentSpeed;
    private double m_winchCurrentSpeed;


    /**
     * This method is called once as soon as the robot is enabled. 
     * Configures our motors by setting them to zero and recording that value
     */
    public void init(){
        m_liftMotor.set(ControlMode.PercentOutput, 0);
        m_liftCurrentSpeed = 0;
        m_winchMotor.set(ControlMode.PercentOutput, 0);
        m_winchCurrentSpeed = 0;
    }

    /**
     * constructor to instantiate climber objects
     */
    public Climber() {
        m_liftMotor = new VictorSPX(RobotMap.ClimberConstants.LIFT_MOTOR_ID);
        m_winchMotor = new VictorSPX(RobotMap.ClimberConstants.WINCH_ID);
    }

     /**
      * sets speed of lift motor to passed in variable
      * @param speed desired percent power for the lift motor (valid values: -1 to 1)
      */
    public void setLiftSpeed(double speed){
        //only sets the power of the motor once any time we change the power to cut down on CAN usage
        if(m_liftCurrentSpeed != speed){
            m_liftCurrentSpeed = speed;
            m_liftMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * sets the speed of winch to passed in variable
     * @param winch desired percent power for the winch motor(valid values 0 to 1)
     */
    public void setWinchSpeed(double speed){
        //only sets the power of the motor once any time we change the power to cut down on CAN usage
        if(m_winchCurrentSpeed != speed){
            m_winchCurrentSpeed = speed;
            m_winchMotor.set(ControlMode.PercentOutput, speed);
        }
    }
}