package frc.robot;

//Import motors
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//Import for sensor 
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {

    // Create Enum to store intake state
    public enum IntakeState {
        kExtended ("Extended"),

        kRetracted ("Retracted"),

        kUnkown ("Unknown");

        private String stateName;


        IntakeState (String stateName){
            this.stateName = stateName; 

        }

        public String toString(){
            return this.stateName;

        }

         
    }
    //Declares motors for the roller bar and interior magazine
    //TODO: rename talonFX to the proper motor that will be used
    private TalonFX m_rollerMotor;
    private TalonFX m_magazineMotor;

    //Declares solenoids for extension and retraction
    private DoubleSolenoid m_solenoid;

    //Delares intake sensor
    private DigitalInput m_magazineSensor;

    //Declares a state enum
    private IntakeState m_state;

    /**
     * Constructor for intake and magazine mechanism
     */
    public Intake(){
        //Instatiate objects for intake class
        //TODO: rename talonFX to the proper motor that will be used
        m_rollerMotor = new TalonFX(RobotMap.IntakeConstants.ROLLER_MOTOR_ID);
        m_magazineMotor = new TalonFX(RobotMap.IntakeConstants.MAGAZINE_MOTOR_ID);

        m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.DOUBLESOLENOID_EXTENDED_PORT);

        m_state = IntakeState.kUnkown;

        m_magazineSensor = new DigitalInput(RobotMap.IntakeConstants.MAGAZINE_SENSOR_PORT);
    }

    /**
     * Initialization method for the intake
     * Sets intake initially to retracted
     */
    public void init(){
        setIntakeExtension(IntakeState.kRetracted);
    }

    /**
     * Activates intake system by powering the roller wheels
     */
    public void takeIn(){
        // Check to see if the intake is extended before activating the front roller motors
        if(m_state == IntakeState.kExtended){
            setFrontRollerSpeed(RobotMap.IntakeConstants.ROLLER_SPEED);
        } 
        else {
            setFrontRollerSpeed(0);
        }
            
    }

    /**
     * Sets the speed of the exterior intake motor
     * @param speed desired speed
     */
    public void setFrontRollerSpeed(double speed){
        m_rollerMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the magazine motor
     * @param speed desired speed
     */
    public void setMagazineSpeed(double speed){
        m_magazineMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Reverses all intake and magazine motors to unjam the robot
     */
    public void unJam(){
        setFrontRollerSpeed(RobotMap.IntakeConstants.REVERSE_ROLLER_SPEED);
        setMagazineSpeed(RobotMap.IntakeConstants.REVERSE_MAGAZINE_SPEED);
    }

    /**
     * sets the intake system between extended/retracted states
     * @param intakeState desired state (intakeExtension.kExtended, intakeExtension.kRetracted)
     */
    public void setIntakeExtension(IntakeState intakeState){
        if (intakeState == m_state){
            return;
        }
        m_state = intakeState;

        if(m_state == IntakeState.kExtended){
            setPiston(Value.kForward);
        }
        else if (m_state == IntakeState.kRetracted){
            setPiston(Value.kReverse);
        }
    }

    /**
     * @return whether or not the intake sensor is being activated
     */
    public boolean checkMagazineSensor() {
        return m_magazineSensor.get();
    }

    /**
     * Sets pistons to a specific value
     * @param value Forward, Reverse
     */
    private void setPiston(DoubleSolenoid.Value value) {
        m_solenoid.set(value);
    }
}
