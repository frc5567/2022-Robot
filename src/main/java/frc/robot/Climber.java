package frc.robot;

// Import motor
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {
    private VictorSPX m_climbMotor;
    private VictorSPX m_winchMotor; 


    /**
     * Initialization method for Climber
     */
    public void init(){
    }

    /**
     * constructor for climber objects
     */
    public Climber() {
        m_climbMotor = new VictorSPX(RobotMap.ClimberConstants.CLIMBER_MOTOR_ID);
        m_winchMotor = new VictorSPX(RobotMap.ClimberConstants.CLIMBER_WINCH_ID);
    }

     /**
      * sets speed of climber to passed in variable
      * @param climb  velocity input (valid values: -1 to 1)
      */
    public void climbCMD(double speed){
        m_climbMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * sets the speed of winch to passed in variable
     * @param winch velocity input (valid values 0 to 1)
     */
    public void winchCMD(double speed){
        m_winchMotor.set(ControlMode.PercentOutput, speed);
    }
}