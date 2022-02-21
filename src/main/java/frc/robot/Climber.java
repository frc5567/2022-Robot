package frc.robot;

// Import motor
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {

private TalonFX m_climbMotor;
private TalonFX m_winchMotor; 

    /**
     * Initialization method for Climber
     */
    public void init(){
    }

    /**
     * constructor for climber objects
     */
    public Climber() {
        m_climbMotor = new TalonFX(RobotMap.ClimberConstants.CLIMBER_MOTOR_ID);
        m_winchMotor = new TalonFX(RobotMap.ClimberConstants.CLIMBER_WINCH_ID);
    }

     /**
      * sets speed of climber to passed in variable
      * @param climb  velocity input (valid values: -1 to 1)
      */
    public void climbCMD(double climb){
        m_climbMotor.set(ControlMode.PercentOutput, climb);
    }

    /**
     * sets the speed of winch to passed in variable
     * @param winch velocity input (valid values 0 to 1)
     */
    public void WinchCMD(double winch){
        m_winchMotor.set(ControlMode.PercentOutput, winch);
    }

}
