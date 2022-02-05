package frc.robot;

// Import pneumatic double solenoid
import edu.wpi.first.wpilibj.DoubleSolenoid;
// This imports an enum that we will call later
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;

// Import motor
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
//Import Motor controller
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.Encoder;

import com.kauailabs.navx.frc.AHRS;

public class Drivetrain {

    // Insert object motor controllers 2 left TalonFX and 2 right TalonFX

    private WPI_TalonFX m_masterRightMotor;
    private WPI_TalonFX m_masterLeftMotor;

    private WPI_TalonFX m_slaveRightMotor;
    private WPI_TalonFX m_slaveLeftMotor;

    // Declare variables for encoders
    private TalonFXSensorCollection m_leftDriveEncoder;
    private TalonFXSensorCollection m_rightDriveEncoder;

    // Pneumatic controller for gear box
    private DoubleSolenoid m_solenoid;

    // Declares the gear used to switch from high to low gear
    private Gear m_gear;

    // Declares the NavX gyro
    private AHRS m_gyro;

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
    
    /**
     * constructor for drivetrain objects
     */
    public Drivetrain () {

        // Instantiate TalonFX Motors
        m_masterRightMotor = new WPI_TalonFX(RobotMap.DrivetrainConstants.MASTER_RIGHT_FALCON_ID);
        m_masterLeftMotor = new WPI_TalonFX(RobotMap.DrivetrainConstants.MASTER_LEFT_FALCON_ID);

        m_slaveRightMotor = new WPI_TalonFX(RobotMap.DrivetrainConstants.SLAVE_RIGHT_FALCON_ID);
        m_slaveLeftMotor = new WPI_TalonFX(RobotMap.DrivetrainConstants.SLAVE_LEFT_FALCON_ID);

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
     * zeroEncoders is a method to set the encoders to 0
     */
    public void zeroEncoders() {
        ErrorCode errorLeft = m_leftDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        ErrorCode errorRight = m_rightDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        
        System.out.println("Resetting encoders. Result: R[" + errorRight.toString() + "] L[" + errorLeft.toString() + "]");
    
    }

    /**
     * zeroGyro is a method to set the gyro to 0
     */
    public void zeroGyro(){
        m_gyro.zeroYaw();
    }

    /**
     * setPiston is a method for changing the state between Forward and reverse
     * @param value what state state to set the pistons to (Forward, Reverse)
     */
    private void setPistons(DoubleSolenoid.Value value) {
        m_solenoid.set(value);
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
            setPistons(Value.kForward);
        }
        else if (m_gear == Gear.kHighGear) {
            setPistons(Value.kReverse);
        }
    }

    /**
     * arcadeDrive is a method for controlling the drivetrain using forward and reverse motion and a turn value
     * @param forward velocity input (valid values: -1 to 1)
     * @param turn rate of turn (valid values: -1 to 1)
     */
    public void arcadeDrive(double forward, double turn){
        m_masterLeftMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, turn);
        m_masterRightMotor.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);

        System.out.println("Arcade Drive: [" + forward + "][" + turn + "]");
    }

    /**
     * Initialization method for the drivetrain
     */
    public void init(){
        zeroEncoders();
        shiftGear(Gear.kHighGear);
        m_masterRightMotor.setInverted(true);
        m_slaveRightMotor.setInverted(true);
        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);
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
