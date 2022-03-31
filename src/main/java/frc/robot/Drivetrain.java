package frc.robot;
// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
// This imports an enum that we will call later
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

// Import motor
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//Import Motor controller
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain {
    // Insert object motor controllers 2 left TalonFX and 2 right TalonFX
    private TalonFX m_masterRightMotor;
    private TalonFX m_masterLeftMotor;

    private TalonFX m_slaveRightMotor;
    private TalonFX m_slaveLeftMotor;

    // Declare variables for encoders
    private TalonFXSensorCollection m_leftDriveEncoder;
    private TalonFXSensorCollection m_rightDriveEncoder;

    // Pneumatic controller for gear box
    private DoubleSolenoid m_solenoid;

    // Declares the gear used to switch from high to low gear
    private Gear m_gear;

    // Declares the NavX gyro
    private AHRS m_gyro;

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
    
    /**
     * constructor for instantiating and creating instances for drivetrain objects:
     * 4 falcon (talonfx) motors, 1 double solenoid, gear, and gyro
     * Configures the turn PID and instantiates PIDTurnController
     */
    public Drivetrain () {

        // Instantiate TalonFX Motors
        m_masterRightMotor = new TalonFX(RobotMap.DrivetrainConstants.MASTER_RIGHT_FALCON_ID);
        m_masterLeftMotor = new TalonFX(RobotMap.DrivetrainConstants.MASTER_LEFT_FALCON_ID);

        m_slaveRightMotor = new TalonFX(RobotMap.DrivetrainConstants.SLAVE_RIGHT_FALCON_ID);
        m_slaveLeftMotor = new TalonFX(RobotMap.DrivetrainConstants.SLAVE_LEFT_FALCON_ID);

        // Instantiate Encoders
        m_leftDriveEncoder = m_masterLeftMotor.getSensorCollection();
        m_rightDriveEncoder = m_masterRightMotor.getSensorCollection(); 

        // Instantiate Solenoid
        m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.DrivetrainConstants.DOUBLESOLENOID_LOW_GEAR_PORT, RobotMap.DrivetrainConstants.DOUBLESOLENOID_HIGH_GEAR_PORT);

        //instantiating the gear and setting it to unknown at the beginning 
        m_gear = Gear.kUnkown; 

        //instantiating the gyro
        m_gyro = new AHRS(SPI.Port.kMXP);
    }

    /**
     * Initialization method for the drivetrain.
     * Brakes all Drivetrain motors, Configures factory default values for all drivetrain motors.
     * Zeros encoders and gyro, Shifts initially to low gear, Inverts all right motors.
     */
    public void init(){
        // Reverts all configurations to factory default values to restore the motor controllers to a known state, allowing you to only config the settings that you intend to change
        m_masterRightMotor.configFactoryDefault();
        m_slaveRightMotor.configFactoryDefault();
        m_masterLeftMotor.configFactoryDefault();
        m_slaveLeftMotor.configFactoryDefault();
        // Zeros encoders initially
        zeroEncoders();
        // Zeros gyro initially
        zeroGyro();
        // Intially starts robot in low gear
        shiftGear(Gear.kLowGear);
        // inverts the right motor so we don't spin in a circle because the motor is flipped
        m_masterRightMotor.setInverted(true);
        // matches inversion for slave motor
        m_slaveRightMotor.setInverted(InvertType.FollowMaster);
    }

    /**
     * periodic is a method for controlling the drivetrain using forward and reverse motion and a turn value
     * @param forward velocity input (valid values: -1 to 1)
     * @param turn rate of turn (valid values: -1 to 1)
     */
    public void periodic(double forward, double turn){
        m_masterLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
        //turn is reversed in order to have opposite power outputs
        m_masterRightMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);

        //System.out.println("Arcade Drive: [" + forward + "][" + turn + "]");
    }

    /**
     * zeroEncoders is a method to set the encoders to 0
     */
    public void zeroEncoders() {
        m_leftDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        m_rightDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
    }

    /**
     * zeroGyro is a method to set the gyro to 0
     */
    public void zeroGyro(){
        //System.out.println("zero gyro");
        m_gyro.zeroYaw();
    }

    /**
     * public method to be used when we want to be able to push the bot around, such as disabled Periodic
     */
    public void coastMode(){
        m_masterRightMotor.setNeutralMode(NeutralMode.Coast);
        m_masterLeftMotor.setNeutralMode(NeutralMode.Coast);
        m_slaveLeftMotor.setNeutralMode(NeutralMode.Coast);
        m_slaveRightMotor.setNeutralMode(NeutralMode.Coast);
    }

    /**
     * public method to be used when we do not want the robot to move unless values are passed in, such as teleop periodic
     */
    public void brakeMode(){
        m_masterRightMotor.setNeutralMode(NeutralMode.Brake);
        m_masterLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveRightMotor.setNeutralMode(NeutralMode.Brake);
    }

    /**
     * shiftGear is a method for changing between Gear.kLowGear and Gear.kHighGear
     * @param gear which gear to change to (Gear.kLowGear, Gear.kHighGear)
     */
    public void shiftGear(Gear gear) {
        if (m_gear == gear) {
            return; 
        }
        m_gear = gear;
        
        if (m_gear == Gear.kLowGear) {
            m_solenoid.set(Value.kForward);
        }
        else if (m_gear == Gear.kHighGear) {
            m_solenoid.set(Value.kReverse);
        }
    }

    /**
     * @return the position of the drivetrain's right encoder
     */
    public double getRightDriveEncoderPosition(){
        return m_masterRightMotor.getSelectedSensorPosition();
    }

    /**
     * @return the position of the drivetrain's left encoder
     */
    public double getLeftDriveEncoderPosition(){
        return m_masterLeftMotor.getSelectedSensorPosition();
    }

    /**
     * @return the angle of the robot in degrees
     */
    public float getGyro(){
        return m_gyro.getYaw();
    }
}
