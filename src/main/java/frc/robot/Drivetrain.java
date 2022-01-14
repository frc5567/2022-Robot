package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
// This imports an enum that we will call later
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

// Import motor
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

//Import Motor controller
import com.ctre.phoenix.motorcontrol.SensorCollection;

public class Drivetrain {

    // Insert object motor controllers 2 left TalonFX and 2 right TalonFX

    private TalonFX m_masterRightMotor;
    private TalonFX m_masterLeftMotor;

    private TalonFX m_slaveRightMotor;
    private TalonFX m_slaveLeftMotor;

    // Declare variables for encoders
    private SensorCollection m_leftDriveEncoder;
    private SensorCollection m_rightDriveEncoder;

    // Pneumatic controller for gear box
    private DoubleSolenoid m_rightSolenoid;
    private DoubleSolenoid m_leftSolenoid;

    // Declares the gear used to switch from high to low gear
    private Gear m_gear;

    // Arcade Drive method 1 trigger to give gas, 1 thumbstick to turn

    // Create Enum for gears
    public enum Gear {
        kLowGear ("Low Gear"),

        kHighGear ("High Gear"),

        kUnkown ("Unknown");

        private String GearName;


        Gear (String GearName){
            this.GearName = GearName; 

        }

        public String toString(){
            return this.GearName;

        }

         
    }
    //constructor for drivetrain objects
    public Drivetrain () {

        // Instantiate TalonFX Motors
        m_masterRightMotor = new TalonFX(RobotMap.MASTER_RIGHT_FALCON_ID);
        m_masterLeftMotor = new TalonFX(RobotMap.MASTER_LEFT_FALCON_ID);

        m_slaveRightMotor = new TalonFX(RobotMap.SLAVE_RIGHT_FALCON_ID);
        m_slaveLeftMotor = new TalonFX(RobotMap.SLAVE_LEFT_FALCON_ID);

        // Instantiate Encoders
        m_leftDriveEncoder = new SensorCollection (m_masterLeftMotor);
        m_rightDriveEncoder = new SensorCollection (m_masterRightMotor); 

        // Instantiate Right and Left Solenoids
        m_rightSolenoid = new DoubleSolenoid(RobotMap.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.LEFT_DOUBLESOLENOID_LOW_GEAR_PORT, RobotMap.LEFT_DOUBLESOLENOID_HIGH_GEAR_PORT);
        m_leftSolenoid  = new DoubleSolenoid(RobotMap.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.RIGHT_DOUBLESOLENOID_LOW_GEAR_PORT, RobotMap.RIGHT_DOUBLESOLENOID_HIGH_GEAR_PORT);

        //instantiating the gear and setting it to unknown at the beginning 
        m_gear = Gear.kUnkown; 
    }

    public void zeroEncoders() {
    
        m_leftDriveEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);
        m_rightDriveEncoder.setQuadraturePosition(0, RobotMap.TIMEOUT_MS);
    }

    private void setPistons(DoubleSolenoid.Value value) {

        m_leftSolenoid.set(value);
        m_rightSolenoid.set(value);
    }



    // TODO: 
    public void shiftGear(Gear gear) {

        if (m_gear == gear) {

            return; 
        }

        m_gear = gear;
        
        if (m_gear == Gear.kLowGear) {

            setPistons(Value.kForward);
        }

        else if (m_gear == Gear.kHighGear) {

            setPistons(Value.kReverse);
        }


    }


    
}
