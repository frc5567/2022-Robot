// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Intake.IntakeState;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private PilotController m_pilotController;
  private CopilotController m_copilotController;
  private Auton m_auton;

  private Intake m_intake;
  private LimelightVision m_limelightVision;
  private Drivetrain m_drivetrain;
  private Launcher m_launcher;
  private Climber m_climber;
  private RobotShuffleboard m_shuffleboard;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    
    m_intake = new Intake();
    m_limelightVision = new LimelightVision();
    m_drivetrain = new Drivetrain();
    m_shuffleboard = new RobotShuffleboard();
    m_launcher = new Launcher(m_limelightVision, m_drivetrain, m_shuffleboard);
    m_climber = new Climber();
    m_shuffleboard.drivetrainShuffleboardConfig();
    
    m_pilotController = new PilotController(m_drivetrain, m_limelightVision, m_shuffleboard, m_launcher, m_climber);
    m_copilotController = new CopilotController(m_intake, m_launcher, m_climber, m_shuffleboard, m_limelightVision);
    m_auton = new Auton(m_drivetrain, m_launcher, m_intake, m_limelightVision, m_shuffleboard);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_drivetrain.brakeMode();
    m_auton.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    m_auton.periodic();
    }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_drivetrain.brakeMode();
    m_pilotController.init();
    m_copilotController.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_pilotController.periodic();
    m_copilotController.periodic();
    m_limelightVision.periodic();
    //m_intake.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_drivetrain.coastMode();
    m_intake.setIntakeExtension(IntakeState.kRetracted);
    m_limelightVision.disableLEDs();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    m_drivetrain.brakeMode();
    m_copilotController.init();
    m_pilotController.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_copilotController.testPeriodic();
    m_pilotController.testPeriodic();
    m_limelightVision.periodic();
    //m_intake.periodic();
  }
}
