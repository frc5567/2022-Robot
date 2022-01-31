package frc.robot;

//Import motors
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
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
    //Declares motors for the roller bar, intake, and interior magazine
    private TalonFX m_frontRollerMotor;
    private TalonFX m_magazineMotor;

    //Declares solenoid for extension and retraction
    private DoubleSolenoid m_Solenoid;

    //Declares a state enum
    private IntakeState m_state;

    /**
     * Constructor for intake and magazine mechanism
     */
    public Intake(){
        m_frontRollerMotor = new TalonFX(RobotMap.IntakeConstants.FRONT_ROLLER_FALCON_ID);
        m_magazineMotor = new TalonFX(RobotMap.IntakeConstants.MAGAZINE_FALCON_ID);

        // Instantiate the Solenoid
        m_Solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.IntakeConstants.DOUBLESOLENOID_RETRACTED_PORT, RobotMap.IntakeConstants.DOUBLESOLENOID_EXTENDED_PORT);

        m_state = IntakeState.kUnkown;
    }

    /**
     * Sets the speed of the exterior intake motor
     * @param speed desired speed
     */
    public void setFrontRollerSpeed(double speed){
        m_frontRollerMotor.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Sets the speed of the magazine motor
     * @param speed desired speed
     */
    public void setMagazineSpeed(double speed){
        m_magazineMotor.set(ControlMode.PercentOutput, speed);
    }
    
    /**
     * Activates intake system by powering both sets of roller wheels
     */
    public void takeIn(){
        setFrontRollerSpeed(RobotMap.IntakeConstants.FRONT_ROLLER_SPEED);
    }

    /**
     * Activates just the magazine
     */
    public void runMagazine(){
        setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
    }

    /**
     * Reverses all intake and magazine motors to unjam the robot
     */
    public void unJam(){
        setFrontRollerSpeed(RobotMap.IntakeConstants.REVERSE_FRONT_ROLLER_SPEED);
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
     * Sets pistons to a specific value
     * @param value Off, Extended, Retracted
     */
    private void setPiston(DoubleSolenoid.Value value) {
        m_Solenoid.set(value);
    }
}
