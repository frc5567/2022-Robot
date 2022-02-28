package frc.robot;

// Import motor
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {
    private WPI_VictorSPX m_climbMotor;
    private WPI_VictorSPX m_winchMotor; 


    /**
     * Initialization method for Climber
     */
    public void init(){
    }

    /**
     * constructor for climber objects
     */
    public Climber() {
        m_climbMotor = new WPI_VictorSPX(RobotMap.ClimberConstants.CLIMBER_MOTOR_ID);
        m_winchMotor = new WPI_VictorSPX(RobotMap.ClimberConstants.CLIMBER_WINCH_ID);
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