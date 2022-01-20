package frc.robot;

//Import motors
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Import Encoders
import com.ctre.phoenix.motorcontrol.SensorCollection;
// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Intake {

    // Create Enum for intake state
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
    //declares motors for the roller bar, intake, and interior magazine
    private TalonFX m_rollerMotor;
    private TalonFX m_intakeMotor;
    private TalonFX m_magazineMotor;

    //declares encoders for intake motors
    private SensorCollection m_rollerEncoder;
    private SensorCollection m_intakeEncoder;
    private SensorCollection m_magazineEncoder;

    //declares solenoids for extension and retraction
    private DoubleSolenoid m_leftSolenoid;
    private DoubleSolenoid m_rightSolenoid;

    private IntakeState m_state;

    /**
     * Constructor for intake and magazine mechanism
     */
    public Intake(){
        m_rollerMotor = new TalonFX(RobotMap.IntakeConstants.ROLLER_FALCON_ID);
        m_intakeMotor = new TalonFX(RobotMap.IntakeConstants.INTAKE_FALCON_ID);
        m_magazineMotor = new TalonFX(RobotMap.IntakeConstants.MAGAZINE_FALCON_ID);

        // Instantiate Right and Left Solenoids
        m_rightSolenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.LEFT_DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.LEFT_DOUBLESOLENOID_EXTENDED_PORT);
        m_leftSolenoid  = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.RIGHT_DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.RIGHT_DOUBLESOLENOID_EXTENDED_PORT);

        m_state = IntakeState.kUnkown;
    }

    //sets the speed of the roller motor
    public void setRollerSpeed(double speed){
        m_rollerMotor.set(ControlMode.PercentOutput, speed);
    }

    //sets the speed of the intake motor
    public void setIntakeSpeed(double speed){
        m_intakeMotor.set(ControlMode.PercentOutput, speed);
    }

    //sets the speed of the magazine motor
    public void setMagazineSpeed(double speed){
        m_magazineMotor.set(ControlMode.PercentOutput, speed);
    }
    
    //Method for activating the intake system
    public void takeIn(){
        setRollerSpeed(RobotMap.IntakeConstants.ROLLER_SPEED);
        setIntakeSpeed(RobotMap.IntakeConstants.INTAKE_SPEED);
    }

    //method for activating the magazine
    public void runMagazine(){
        setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
    }

    //method for reversing all intake/magazine motors to unjam game pieces
    public void unJam(){
        setRollerSpeed(RobotMap.IntakeConstants.REVERSE_ROLLER_SPEED);
        setIntakeSpeed(RobotMap.IntakeConstants.REVERSE_INTAKE_SPEED);
        setMagazineSpeed(RobotMap.IntakeConstants.REVERSE_MAGAZINE_SPEED);
    }

    //method for toggling between extended and retracted modes on the intake
    public void toggleIntakeExtension(IntakeState intakeState){
        if (intakeState == m_state){
            return;
        }
        m_state = intakeState;

        if(m_state == IntakeState.kExtended){
            setPistons(Value.kForward);
        }
        else if (m_state == IntakeState.kRetracted){
            setPistons(Value.kReverse);
        }

    }

    //method for setting the pistons to a given value
    private void setPistons(DoubleSolenoid.Value value) {
        m_leftSolenoid.set(value);
        m_rightSolenoid.set(value);
    }
}
