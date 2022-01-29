package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
import com.ctre.phoenix.motorcontrol.FollowerType;
//Import Motor controller
import com.ctre.phoenix.motorcontrol.SensorCollection;

import com.kauailabs.navx.frc.AHRS;

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
    //constructor for drivetrain objects
    public Drivetrain () {

        // Instantiate TalonFX Motors
        m_masterRightMotor = new TalonFX(RobotMap.DrivetrainConstants.MASTER_RIGHT_FALCON_ID);
        m_masterLeftMotor = new TalonFX(RobotMap.DrivetrainConstants.MASTER_LEFT_FALCON_ID);

        m_slaveRightMotor = new TalonFX(RobotMap.DrivetrainConstants.SLAVE_RIGHT_FALCON_ID);
        m_slaveLeftMotor = new TalonFX(RobotMap.DrivetrainConstants.SLAVE_LEFT_FALCON_ID);

        // Instantiate Encoders
        m_leftDriveEncoder = new SensorCollection (m_masterLeftMotor);
        m_rightDriveEncoder = new SensorCollection (m_masterRightMotor); 

        // Instantiate Solenoid
        m_solenoid = new DoubleSolenoid(RobotMap.CANConstants.PCM_CAN_ID, PneumaticsModuleType.CTREPCM, RobotMap.DrivetrainConstants.DOUBLESOLENOID_LOW_GEAR_PORT, RobotMap.DrivetrainConstants.DOUBLESOLENOID_HIGH_GEAR_PORT);

        //instantiating the gear and setting it to unknown at the beginning 
        m_gear = Gear.kUnkown; 

        //instantiating the gyro
        m_gyro = new AHRS(SPI.Port.kMXP);   
    }

    public void zeroEncoders() {
    
        m_leftDriveEncoder.setQuadraturePosition(0, RobotMap.DrivetrainConstants.TIMEOUT_MS);
        m_rightDriveEncoder.setQuadraturePosition(0, RobotMap.DrivetrainConstants.TIMEOUT_MS);
    }

    public void zeroGyro(){
        m_gyro.zeroYaw();
    }
    
    private void setPistons(DoubleSolenoid.Value value) {
        m_solenoid.set(value);
    }

    /**
     * shiftGear is a method for changing between low and high gears
     * @param gear which gear to change to
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

        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);
    }

    
    public void init(){
        zeroEncoders();
        shiftGear(Gear.kHighGear);
        m_masterRightMotor.setInverted(true);
        m_slaveRightMotor.setInverted(true);
    }

    /**
     * returns the position of the drivetrain's right encoder
     */
    public double getRightDriveEncoderPosition(){
        return m_masterRightMotor.getSelectedSensorPosition();
    }

    /**
     * returns the position of the drivetrain's left encoder
     */
    public double getLeftDriveEncoderPosition(){
        return m_masterLeftMotor.getSelectedSensorPosition();
    }

    /**
     * returns the angle of the robot in degrees
     */
    public float getGyro(){
        return m_gyro.getYaw();
    }
}
