package frc.robot;

//Import motors
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

//Import for sensor 
import edu.wpi.first.wpilibj.DigitalInput;

public class Intake {

    /**
     * Enum to store intake state Can be kExtdended, kRetracted, or KUnkown.
     */ 
    public enum IntakeState {
        //When the intake bar is outside of the frame perimiter
        kExtended ("Extended"),

        //When the intake bar is inside the frame perimiter
        kRetracted ("Retracted"),

        //When we do not know the state of the intake 
        kUnkown ("Unknown");

        private String stateName;


        IntakeState (String stateName){
            this.stateName = stateName; 

        }

        public String toString(){
            return this.stateName;

        }

         
    }
    //Declares motors for the roller bar and interior magazine (carwash) motors
    private VictorSPX m_rollerMotor;
    private VictorSPX m_magazineMotor;

    //Declares solenoid for extension and retraction of the intake bar
    private DoubleSolenoid m_solenoid;

    /**
     * Sensor before magazine motors
     */
    private DigitalInput m_sensor0;

    /**
     * Sensor after magazine motors
     */
    private DigitalInput m_sensor1;

    //Declares a state enum
    private IntakeState m_state;

    //Member variables used to make sure we aren't calling on the CAN bus constantly by repetedly setting the motors to zero 
    private double m_rollerCurrentSpeed;
    private double m_magazineCurrentSpeed;

    /**
     * Constructor for intake and magazine mechanism
     */
    public Intake(){
        //Instatiate objects for intake class
        m_rollerMotor = new VictorSPX(RobotMap.IntakeConstants.ROLLER_MOTOR_ID);
        m_magazineMotor = new VictorSPX(RobotMap.IntakeConstants.MAGAZINE_MOTOR_ID);

        m_solenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.DOUBLESOLENOID_EXTENDED_PORT);

        m_state = IntakeState.kUnkown;

        m_sensor0 = new DigitalInput(RobotMap.IntakeConstants.MAGAZINE_SENSOR_0_PORT);
        m_sensor1 = new DigitalInput(RobotMap.IntakeConstants.MAGAZINE_SENSOR_1_PORT);
    }

    /**
     * This method is called once as soon as the robot is enabled. 
     * Sets intake initially to retracted and configures our motors
     */
    public void init(){
        setIntakeExtension(IntakeState.kRetracted);
        m_rollerMotor.set(ControlMode.PercentOutput, 0);
        m_rollerCurrentSpeed = 0;
        m_magazineMotor.set(ControlMode.PercentOutput, 0);
        m_magazineCurrentSpeed = 0;
        m_rollerMotor.setInverted(true);
    }

    /**
     * This method is called many times a second in robotPeriodic.
     * Automatically moves game pieces from the first position to the second position if possible
     */
    public void indexing(){
        //If a game piece is in the first slot, and there is no game piece in the second slot, move the game piece to the second slot

        boolean currentlyIndexing = false;
        if(getMagazineSensor0()){
            if(!getMagazineSensor1()){
                currentlyIndexing = true;
            }
        }

        if(currentlyIndexing){
            setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
        }
        else{
            setMagazineSpeed(0);
        }
    }
    

    /**
     * Activates intake system by powering the roller wheels
     */
    public void takeIn(double speed){
        // Check to see if the intake is extended before activating the front roller motor
        if(m_state == IntakeState.kExtended){
            setRollerSpeed(speed);
        } 
        else {
            setRollerSpeed(0);
        }
            
    }

    /**
     * Sets the speed of the exterior intake motor
     * @param speed desired speed of the roller bar
     */
    public void setRollerSpeed(double speed){
        //only sets the power of the motor once any time we change the power to cut down on CAN usage
        if(m_rollerCurrentSpeed != speed){
            m_rollerCurrentSpeed = speed;
            m_rollerMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the magazine motor
     * @param speed desired speed of the magazine (carwash) motor
     */
    public void setMagazineSpeed(double speed){
        //only sets the power of the motor once any time we change the power to cut down on CAN usage
        if(m_magazineCurrentSpeed != speed){
            m_magazineCurrentSpeed = speed;
            m_magazineMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * sets the intake system between extended/retracted states
     * @param intakeState desired state (intakeExtension.kExtended, intakeExtension.kRetracted)
     */
    public void setIntakeExtension(IntakeState intakeState){
        //If we are already in the state we pass in, keep the state the same
        if (intakeState == m_state){
            return;
        }
        //Set the m_state member variable to the state we pass in
        m_state = intakeState;

        //Set the pistons on the robot to the state passed into this method
        if(m_state == IntakeState.kExtended){
            m_solenoid.set(Value.kForward);
        }
        else if (m_state == IntakeState.kRetracted){
            m_solenoid.set(Value.kReverse);
        }
    }

    /**
     * @return whether or not the first magazine sensor is being activated: True if beam is broken, False if not
     */
    public boolean getMagazineSensor0() {
        return !m_sensor0.get();
    }

    /**
     * @return whether or not the second magazine sensor is being activated: True if beam is broken, False if not
     */
    public boolean getMagazineSensor1() {
        return !m_sensor1.get();
    }
}
