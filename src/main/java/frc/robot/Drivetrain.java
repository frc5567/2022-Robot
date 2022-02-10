package frc.robot;

import edu.wpi.first.math.controller.PIDController;
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
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
//Import Motor controller
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import edu.wpi.first.wpilibj.Encoder;

import com.kauailabs.navx.frc.AHRS;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

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

    private PIDController m_PIDTurnController;

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

        //configSensor();

        // Instantiate PID Controller
        m_PIDTurnController = new PIDController(RobotMap.DrivetrainConstants.TURN_GAINS.kP, RobotMap.DrivetrainConstants.TURN_GAINS.kI, RobotMap.DrivetrainConstants.TURN_GAINS.kD);

    }

    /**
     * zeroEncoders is a method to set the encoders to 0
     */
    public void zeroEncoders() {
        //ErrorCode errorLeft = m_leftDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        //ErrorCode errorRight = m_rightDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);

        m_leftDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        m_rightDriveEncoder.setIntegratedSensorPosition(0, RobotMap.TIMEOUT_MS);
        
        //System.out.println("Resetting encoders. Result: R[" + errorRight.toString() + "] L[" + errorLeft.toString() + "]");
    
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
        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);

        System.out.println("Arcade Drive: [" + forward + "][" + turn + "]");
    }

    /**
     * Initialization method for the drivetrain
     */
    public void init(){
        m_masterRightMotor.setNeutralMode(NeutralMode.Brake);
        m_masterLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveLeftMotor.setNeutralMode(NeutralMode.Brake);
        m_slaveRightMotor.setNeutralMode(NeutralMode.Brake);
        m_masterRightMotor.configFactoryDefault();
        m_slaveRightMotor.configFactoryDefault();
        m_masterLeftMotor.configFactoryDefault();
        m_slaveLeftMotor.configFactoryDefault();
        zeroEncoders();
        shiftGear(Gear.kHighGear);
        m_masterRightMotor.setInverted(true);
        m_slaveRightMotor.setInverted(InvertType.FollowMaster);
        m_slaveLeftMotor.setInverted(InvertType.FollowMaster);
        m_slaveLeftMotor.follow(m_masterLeftMotor);
        m_slaveRightMotor.follow(m_masterRightMotor);
    }

    /**
     * @return the position of the drivetrain's right encoder
     */
    public double getRightDriveEncoderPosition(){
        return m_masterRightMotor.getSelectedSensorPosition();
        //return m_rightDriveEncoder.getIntegratedSensorPosition();
    }

    /**
     * @return the position of the drivetrain's left encoder
     */
    public double getLeftDriveEncoderPosition(){
        return m_masterLeftMotor.getSelectedSensorPosition();
        //return m_leftDriveEncoder.getIntegratedSensorPosition();
    }

    public void configSensor(){
        m_masterLeftMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, RobotMap.TIMEOUT_MS);
        m_masterRightMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, RobotMap.TIMEOUT_MS);

    }

    /**
     * @return the angle of the robot in degrees
     */
    public float getGyro(){
        return m_gyro.getYaw();
    }
    
    /**
     * This class configures the PID controller and allows it to recieve continuous input as well as setting the 
     * integrator range to stop us from turning too far and it also sets the tolerance to determine in we need to stop or replan the input.
     */
    private void PIDConfig(){
        m_PIDTurnController.enableContinuousInput(-RobotMap.DrivetrainConstants.PID_INPUT_RANGE, RobotMap.DrivetrainConstants.PID_INPUT_RANGE);
        m_PIDTurnController.setIntegratorRange(-RobotMap.DrivetrainConstants.ROTATE_PID_INTEGRATOR_RANGE, RobotMap.DrivetrainConstants.ROTATE_PID_INTEGRATOR_RANGE);
        m_PIDTurnController.setTolerance(RobotMap.DrivetrainConstants.TOLERANCE_ROTATE_CONTROLLER);
    }

}
