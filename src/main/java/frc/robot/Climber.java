package frc.robot;

// Import motor
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Climber {
    private VictorSPX m_climbMotor;
    private VictorSPX m_winchMotor; 

    private double m_climberCurrentSpeed;
    private double m_winchCurrentSpeed;


    /**
     * Initialization method for Climber
     */
    public void init(){
        m_climbMotor.set(ControlMode.PercentOutput, 0);
        m_climberCurrentSpeed = 0;
        m_winchMotor.set(ControlMode.PercentOutput, 0);
        m_winchCurrentSpeed = 0;
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
    public void setLiftSpeed(double speed){
        if(m_climberCurrentSpeed != speed){
            m_climberCurrentSpeed = speed;
            m_climbMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * sets the speed of winch to passed in variable
     * @param winch velocity input (valid values 0 to 1)
     */
    public void setWinchSpeed(double speed){
        if(m_winchCurrentSpeed != speed){
            m_winchCurrentSpeed = speed;
            m_winchMotor.set(ControlMode.PercentOutput, speed);
        }
    }
}