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
    private VictorSPX m_rollerMotor;
    private VictorSPX m_magazineMotor;

    //Declares solenoids for extension and retraction
    private DoubleSolenoid m_solenoid;

    //Delares the first magazine sensor that is before the "carwash" wheels
    private DigitalInput m_sensor1;

    //Delares the second magazine sensor that is after the "carwash" wheels
    private DigitalInput m_sensor2;

    //Declares a state enum
    private IntakeState m_state;

    private double m_rollerCurrentSpeed;
    private double m_magazineCurrentSpeed;

    /**
     * Constructor for intake and magazine mechanism
     */
    public Intake(){
        //Instatiate objects for intake class
        m_rollerMotor = new VictorSPX(RobotMap.IntakeConstants.ROLLER_MOTOR_ID);
        m_magazineMotor = new VictorSPX(RobotMap.IntakeConstants.MAGAZINE_MOTOR_ID);

        m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.DOUBLESOLENOID_EXTENDED_PORT);

        m_state = IntakeState.kUnkown;

        m_sensor1 = new DigitalInput(RobotMap.IntakeConstants.MAGAZINE_SENSOR_0_PORT);
        m_sensor2 = new DigitalInput(RobotMap.IntakeConstants.MAGAZINE_SENSOR_1_PORT);
    }

    /**
     * Initialization method for the intake
     * Sets intake initially to retracted
     */
    public void init(){
        //setIntakeExtension(IntakeState.kRetracted);
        m_rollerMotor.set(ControlMode.PercentOutput, 0);
        m_rollerCurrentSpeed = 0;
        m_magazineMotor.set(ControlMode.PercentOutput, 0);
        m_magazineCurrentSpeed = 0;
    }

    /**
     * This method is called many times a second in robotPeriodic. It is currently only used for automatic indexing
     */
    public void periodic(){
        //If a game piece is in the first slot, and there is no game piece in the second slot, move the game piece to the second slot
        if(getMagazineSensor1()){
            if(getMagazineSensor2()){
                return;
            }
            else{
                setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
            }
        }
    }
    

    /**
     * Activates intake system by powering the roller wheels
     */
    public void takeIn(){
        // Check to see if the intake is extended before activating the front roller motors
        if(m_state == IntakeState.kExtended){
            setRollerSpeed(RobotMap.IntakeConstants.ROLLER_SPEED);
        } 
        else {
            setRollerSpeed(0);
        }
            
    }

    /**
     * Sets the speed of the exterior intake motor
     * @param speed desired speed
     */
    public void setRollerSpeed(double speed){
        if(m_rollerCurrentSpeed != speed){
            m_rollerCurrentSpeed = speed;
            m_rollerMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Sets the speed of the magazine motor
     * @param speed desired speed
     */
    public void setMagazineSpeed(double speed){
        if(m_magazineCurrentSpeed != speed){
            m_magazineCurrentSpeed = speed;
            m_magazineMotor.set(ControlMode.PercentOutput, speed);
        }
    }

    /**
     * Reverses all intake and magazine motors to unjam the robot
     */
    public void unJam(){
        setRollerSpeed(RobotMap.IntakeConstants.REVERSE_ROLLER_SPEED);
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
     * @return whether or not the first magazine sensor is being activated
     */
    public boolean getMagazineSensor1() {
        return m_sensor1.get();
    }

    /**
     * @return whether or not the second magazine sensor is being activated
     */
    private boolean getMagazineSensor2() {
        return m_sensor2.get();
    }

    /**
     * Sets pistons to a specific value
     * @param value Value.kForward, Value.kReverse
     */
    private void setPiston(DoubleSolenoid.Value value) {
        m_solenoid.set(value);
    }
}
