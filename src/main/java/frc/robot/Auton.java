package frc.robot;

import frc.robot.Intake.IntakeState;

public class Auton{
    /**
     * Enum for storing what path we are going to take in auton:
     * kFourBall (Not currently entirely tested and finished), kThreeBall (start in right tarmac), and kTwoBall
     */
    public enum AutonPath{

        //auton path for starting on the tarmac lines on the right side from the drivers' station perspective and shoot four balls
        kFourBall,

        //Auton path for starting on the tarmac lines on the right side from the drivers' station perspective and shoot three balls
        kThreeBall,

        //auton path for starting lined up directly with a ball
        kTwoBall;
    }

    /**
     * enum for each of the steps in our auton (k1-k18 and kStop)
     */
    public enum AutonStep{
        kStep1,

        kStep2,

        kStep3,

        kStep4,

        kStep5,

        kStep6,

        kStep7,

        kStep8,

        kStep9,

        kStep10,

        kStep11,

        kStep12,

        kStep13,

        kStep14,

        kStep15,

        kStep16,
        
        kStep17,

        kStep18,

        kStop;
    }
    
    //This stores the loop count to make sure the intake is not retracted before one of the two balls leaves the robot. Prevents jamming
    private int m_intakeInLoopCount = 0;
    //Member variable for a loop count to make sure the robot does not move until the balls have been fully launched
    private int m_loopsAfterLaunchCount = 0;

    //declares variables for the auton class
    //Enums to store what step we are on and what path we chose
    private AutonStep m_step;
    private AutonPath m_path;

    //Member variables to store the systems we pass in (drivetrain, launcher, intake, and limelight)
    private Drivetrain m_drivetrain;
    private Launcher m_launcher;
    private Intake m_intake;
    private LimelightVision m_limelightVision;

    //declares shuffleboard to be used for path selection
    private RobotShuffleboard m_shuffleboard;
    //Initially sets/defaults our auton path to be the two ball auton (2 ball == 2, 3 ball == 3)
    private double m_currentAutonPath = RobotMap.ShuffleboardConstants.DEFAULT_AUTON_PATH;

    //Varaible that is only true if we have a second ball to launch
    boolean m_secondBall = false;
    //Variable that is only true the first time through the auton periodic loop to print out that we have started auton
    boolean m_autonStartFlag = true;
    //Variable to store the current state of our Limelight LEDS
    boolean m_limelightOff = true;
    //Boolean for Sysout counter
    boolean m_doSysOut = true;

    //Stores the value from the xAngleToTarget method (value is the x angle offset from the target)
    double m_xToTarget;

    //Member variable for the sysOutCounter to prevent the console from being flooded
    int m_sysOutCounter;

    /**
     * constructor for auton to instantiate all of the objects and set our Auton step initially to step 1
     * @param drivetrain we pass in drivetrain to be able to drive around in auton
     * @param launcher we pass in the launcher to be able to launch during auton
     * @param intake we pass in the intake to be able to pick up game pieces during auton
     * @param limelightVision we pass in limelight to be able to target during auton
     */
    public Auton(Drivetrain drivetrain, Launcher launcher, Intake intake, LimelightVision limelightVision, RobotShuffleboard shuffleboard){
        m_shuffleboard = shuffleboard;
        m_drivetrain = drivetrain;
        m_launcher = launcher;
        m_intake = intake;
        m_limelightVision = limelightVision;

        //Sets the current step to step 1 and the path to whatever path we are currently running
        m_step = AutonStep.kStep1;
    }

    /**
     * this method will be run at the start of every auton period (called in autonomousInit() in Robot.java).
     * This initally selects which auton path we will be running through the shuffleboard.
     */
    public void init(){
        m_drivetrain.init();
        m_launcher.init();
        m_intake.init();
        m_limelightVision.init();
        m_step = AutonStep.kStep1;   
        m_currentAutonPath = m_shuffleboard.getAutonPath();
        selectPath();
    }

    /**
     * this method will be called many times a second during the auton period
     */
    public void periodic(){
        
        //calls the limelight periodic method in order to update the network tables every cycle
        m_limelightVision.periodic();

        //Variable to store the status of the leds on the limelight
        int CurrentLEDStatus = m_limelightVision.currentLEDStatus();
        //If the Limelight is off, sets m_limelightOff to be true
        if(CurrentLEDStatus == 1){
            m_limelightOff = true;
        }
        //If the Limelight is on, sets m_limelightOff to be false
        else if(CurrentLEDStatus == 3){
            m_limelightOff = false;
        }
        else{
            System.out.println("unknown LED status");
        }

        //Prints out "Starting Auton" only once
        if(m_autonStartFlag){
            System.out.println("STARTING AUTON");
            m_autonStartFlag = false;
        }
 
        //Counter for sysouts
        if((m_sysOutCounter++ % 20) == 0){
            m_doSysOut = true;
        }
        else{
            m_doSysOut = false;
        }

        //Prints out the current Auton step, path, and status of the magazine sensors
        if(m_doSysOut == true){
            // System.out.println("Current Angle: " + m_drivetrain.getGyro());
            System.out.println("Current Step:" + m_step + " Current Path:" + m_path);
            System.out.println("First Sensor   [" + m_intake.getMagazineSensor0() + "] --- " + "Second Sensor    [" + m_intake.getMagazineSensor1() + "]");
        }

        //Starts auton pathing. Our paths are Two ball and Four ball
        /**
         * This path will have our robot launch 2 balls during auton.
         * The robot can start in either tarmac as close to the front of it as possible while being centered on and pointed at the ball
         */
        if (m_path == AutonPath.kTwoBall){ 
            //Activates intake and drives forward to a target in order to pick up a game piece
            if(m_step == AutonStep.kStep1){
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.TWO_BALL_STEP_ONE_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //This step uses a loop count to have the robot intake for 30 loops to make sure the ball is actually picked up
            else if(m_step == AutonStep.kStep2){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep3;
                }
            }
            //step to Turn the robot 179 degrees in order to face the hub.
            else if(m_step == AutonStep.kStep3){
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.TWO_BALL_STEP_FOUR_TARGET_ANGLE)){
                    //Zeros the drivetrain when we are at our desired angle so we don't keep spinning.
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep4;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            //step to turn the robot roughly to the target to be fine tuned by the turret
            else if(m_step == AutonStep.kStep4){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                System.out.println("Activating Limelight");
                m_limelightVision.enableLEDs();
                //Stores the current angle from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen (within 5 degrees on both sides), send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0,0);
                        m_drivetrain.zeroEncoders();
                        m_step = AutonStep.kStep5;
                        if(m_doSysOut == true){
                            System.out.println("On Target");
                        }
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target and move left so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move right so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                            System.out.println("Ta " + m_limelightVision.tAreaOfScreen() + "   Tx " + m_limelightVision.xAngleToTarget() + "   Ty " + m_limelightVision.yAngleToTarget());
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            //step to aim with the turret and launch
            else if (m_step == AutonStep.kStep5){ 
                m_launcher.targetAndLaunch();
                //This loop count is used to make sure both balls have been fully launched before moving on to the next step
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.LOOPS_AFTER_LAUNCH){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep6;
                }
            }
            //step to move backwards, out of tarmac and zero everything other than drivetrain. We also retract the intake
            else if (m_step == AutonStep.kStep6){
                m_launcher.setFeederSpeed(0);
                m_launcher.setTurretSpeed(0);
                m_launcher.setFlywheelSpeed(0);
                m_intake.setRollerSpeed(0);
                m_intake.setMagazineSpeed(0);
                m_limelightVision.disableLEDs();
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                if(driveToTarget(-RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.TWO_BALL_STEP_SIX_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep7;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
/**
 * *******************************  This is how far I got in checking and cleaning up the Auton Code *********************************
 * 
 */
            }
            // In this step we turn 179 degrees around to orient ourself for teleop
            else if (m_step == AutonStep.kStep7){
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.TWO_BALL_STEP_NINE_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStop;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();                }
                else{
                    return;
                }
            }
            //step to finish zeroing everything for the end of auton
            else if(m_step == AutonStep.kStop){
                if(m_doSysOut == true){
                    System.out.println("Auton Completed");
                }
                m_drivetrain.periodic(0, 0);
            }
        }
        else if (m_path == AutonPath.kThreeBall){
            //Activates intake and drives forward to a target in order to pick up a game piece
            if(m_step == AutonStep.kStep1){
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
                // m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.THREE_BALL_STEP_ONE_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //step to retract intake if build finds a way to do this
            else if(m_step == AutonStep.kStep2){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep3;
                }
            }
            //step to Turn the robot a full 180 degrees in order to face the hub. Also brings ball up to the feeder.
            else if(m_step == AutonStep.kStep3){
                // m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.THREE_BALL_STEP_FOUR_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep4;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            //step to turn robot roughly to the target to be fine tuned by the turret
            else if(m_step == AutonStep.kStep4){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                System.out.println("Activating Limelight");
                m_limelightVision.enableLEDs();
                //Stores the current distance from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen, send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0,0);
                        m_drivetrain.zeroEncoders();
                        m_step = AutonStep.kStep5;
                        if(m_doSysOut == true){
                            System.out.println("On Target");
                            System.out.println("Ta " + m_limelightVision.tAreaOfScreen() + "   Tx " + m_limelightVision.xAngleToTarget() + "   Ty " + m_limelightVision.yAngleToTarget());
                        }
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                            System.out.println("Ta " + m_limelightVision.tAreaOfScreen() + "   Tx " + m_limelightVision.xAngleToTarget() + "   Ty " + m_limelightVision.yAngleToTarget());
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            //step to aim with the turret and launch
            else if (m_step == AutonStep.kStep5){
                m_launcher.targetAndLaunch();

                if(m_intake.getMagazineSensor0() || m_intake.getMagazineSensor1()){
                    m_loopsAfterLaunchCount = 0;
                }

                if(!(m_intake.getMagazineSensor1())){
                    m_intake.setIntakeExtension(IntakeState.kRetracted);
                    
                    if(!(m_intake.getMagazineSensor0())){
                        System.out.println("BOTH SENSORS ARE EMPTY --- COUNT IS AT " + m_loopsAfterLaunchCount);
                        m_loopsAfterLaunchCount++;
                    }
                }

                if(m_loopsAfterLaunchCount >= 15){
                    m_loopsAfterLaunchCount = 0;
                    m_step = AutonStep.kStep6;
                }
            }
            //step to move backwards, out of tarmac and zero everything other than drivetrain. We also retract the intake
            else if (m_step == AutonStep.kStep6){
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_launcher.setFeederSpeed(0);
                m_launcher.setTurretSpeed(0);
                m_launcher.setFlywheelSpeed(0);
                m_intake.setRollerSpeed(0);
                m_intake.setMagazineSpeed(0);
                m_limelightVision.disableLEDs();
                m_drivetrain.periodic(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                m_step = AutonStep.kStep7;
            }
            //step to turn 45 degrees counter clockwise around to orient ourself for the terminal ball
            else if (m_step == AutonStep.kStep7){
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.THREE_BALL_STEP_NINE_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep8;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();               
                }
                else{
                    return;
                }
            }
            //step to drive to the other ball
            else if (m_step == AutonStep.kStep8){
                if(driveToTarget(0.7, RobotMap.AutonConstants.THREE_BALL_STEP_TEN_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep9;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep9){
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
                m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.THREE_BALL_STEP_ELEVEN_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep10;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep10){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep11;
                }
            }
            //step to turn towards the hub again
            else if (m_step == AutonStep.kStep11){
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_intake.takeIn(0);
                m_intake.setMagazineSpeed(0);
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.THREE_BALL_STEP_THIRTEEN_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep12;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();                
                }
                else{
                    return;
                }
            }
            else if (m_step == AutonStep.kStep12){
                m_limelightVision.enableLEDs();
                m_launcher.targetAndLaunch();

                if(m_intake.getMagazineSensor0() || m_intake.getMagazineSensor1()){
                    m_loopsAfterLaunchCount = 0;
                }

                if(!(m_intake.getMagazineSensor1())){
                    m_intake.setIntakeExtension(IntakeState.kRetracted);
                    
                    if(!(m_intake.getMagazineSensor0())){
                        System.out.println("BOTH SENSORS ARE EMPTY --- COUNT IS AT " + m_loopsAfterLaunchCount);
                        m_loopsAfterLaunchCount++;
                    }
                }

                if(m_loopsAfterLaunchCount >= 15){
                    m_loopsAfterLaunchCount = 0;
                    m_step = AutonStep.kStop;
                }
            }
            else if (m_step == AutonStep.kStop){
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_launcher.setFeederSpeed(0);
                m_launcher.setTurretSpeed(0);
                m_launcher.setFlywheelSpeed(0);
                m_intake.setRollerSpeed(0);
                m_intake.setMagazineSpeed(0);
                m_limelightVision.disableLEDs();
                m_drivetrain.periodic(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
            }

        }
        else if (m_path == AutonPath.kFourBall){ 
            //Activates intake and drives forward to a target in order to pick up a game piece
            if(m_step == AutonStep.kStep1){
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
                // m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_ONE_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep2;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //step to retract intake if build finds a way to do this
            else if(m_step == AutonStep.kStep2){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep3;
                }
            }
            //step to Turn the robot a full 180 degrees in order to face the hub. Also brings ball up to the feeder.
            else if(m_step == AutonStep.kStep3){
                // m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_FOUR_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep4;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();
                }
                else{
                    return;
                }
            }
            //step to turn robot roughly to the target to be fine tuned by the turret
            else if(m_step == AutonStep.kStep4){
                //If the limelight is currently not switched on, turn it on. Otherwise, keep it on
                System.out.println("Activating Limelight");
                m_limelightVision.enableLEDs();
                //Stores the current distance from the target to the center of the screen on the x axis in a variable
                m_xToTarget = m_limelightVision.xAngleToTarget();
                if(m_limelightVision.seeTarget()){
                    //If the target is close enough to the center of the screen, send a print out to the driver station, stop the robot, turn of the limelight LEDS, and reset encoders
                    if(m_xToTarget < RobotMap.TOLERATED_TARGET_ERROR && m_xToTarget > -RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0,0);
                        m_drivetrain.zeroEncoders();
                        m_step = AutonStep.kStep5;
                        if(m_doSysOut == true){
                            System.out.println("On Target");
                            System.out.println("Ta " + m_limelightVision.tAreaOfScreen() + "   Tx " + m_limelightVision.xAngleToTarget() + "   Ty " + m_limelightVision.yAngleToTarget());
                        }
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget > RobotMap.TOLERATED_TARGET_ERROR){
                        m_drivetrain.periodic(0, RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                    //If the target is not close enough to the center of the screen, print out that we are not on target move so that it is
                    else if(m_xToTarget < -RobotMap.TOLERATED_TARGET_ERROR){
                        if(m_doSysOut == true){
                            System.out.println("Not On Target");
                            System.out.println("Ta " + m_limelightVision.tAreaOfScreen() + "   Tx " + m_limelightVision.xAngleToTarget() + "   Ty " + m_limelightVision.yAngleToTarget());
                        }
                        m_drivetrain.periodic(0, -RobotMap.AutonConstants.TARGETING_SPEED);
                    }
                }
                else{
                    if(m_doSysOut == true){
                        System.out.println("No Target Detected");
                    }
                }
            }
            //step to aim with the turret and launch
            else if (m_step == AutonStep.kStep5){
                m_launcher.targetAndLaunch();
                // m_intakeInLoopCount ++;
                // if(m_intakeInLoopCount >= 40){
                //     m_intake.setIntakeExtension(IntakeState.kRetracted);
                //     m_intakeInLoopCount = 0;
                // }

                if(m_intake.getMagazineSensor0() || m_intake.getMagazineSensor1()){
                    m_loopsAfterLaunchCount = 0;
                }

                if(!(m_intake.getMagazineSensor1())){
                    m_intake.setIntakeExtension(IntakeState.kRetracted);
                    
                    if(!(m_intake.getMagazineSensor0())){
                        System.out.println("BOTH SENSORS ARE EMPTY --- COUNT IS AT " + m_loopsAfterLaunchCount);
                        m_loopsAfterLaunchCount++;
                    }
                }

                if(m_loopsAfterLaunchCount >= 15){
                    m_loopsAfterLaunchCount = 0;
                    m_step = AutonStep.kStep6;
                }
            }
            //step to move backwards, out of tarmac and zero everything other than drivetrain. We also retract the intake
            else if (m_step == AutonStep.kStep6){
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_launcher.setFeederSpeed(0);
                m_launcher.setTurretSpeed(0);
                m_launcher.setFlywheelSpeed(0);
                m_intake.setRollerSpeed(0);
                m_intake.setMagazineSpeed(0);
                m_limelightVision.disableLEDs();
                m_drivetrain.periodic(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                m_step = AutonStep.kStep7;
            }
            //step to turn 110 degrees counter clockwise around to orient ourself for the terminal ball
            else if (m_step == AutonStep.kStep7){
                if(turnToAngle(-RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_NINE_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep8;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();               
                }
                else{
                    return;
                }
            }
            //step to drive to the terminal ball 
            else if (m_step == AutonStep.kStep8){
                if(driveToTarget(0.7, RobotMap.AutonConstants.FOUR_BALL_STEP_TEN_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep9;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            else if(m_step == AutonStep.kStep9){
                m_intake.setIntakeExtension(IntakeState.kExtended);
                m_intake.takeIn(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(driveToTarget(RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_ELEVEN_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep10;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //step to retract intake if build finds a way to do this
            else if(m_step == AutonStep.kStep10){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep11;
                }
            }
            //step to move backwards after intaking the terminal ball in order to not hit the wall  
            else if (m_step == AutonStep.kStep11){
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                m_intake.takeIn(0);
                m_intake.setMagazineSpeed(0);
                if(driveToTarget(-RobotMap.AutonConstants.DRIVE_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_TWELVE_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep12;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //step to turn towards the hub again
            else if (m_step == AutonStep.kStep12){
                if(turnToAngle(RobotMap.AutonConstants.TURN_SPEED, RobotMap.AutonConstants.FOUR_BALL_STEP_THIRTEEN_TARGET_ANGLE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep13;
                    m_drivetrain.zeroEncoders();
                    m_drivetrain.zeroGyro();                
                }
                else{
                    return;
                }
            }
            //step to drive towards the hub intaking the ball along the right line
            else if (m_step == AutonStep.kStep13){
                // m_intake.setIntakeExtension(IntakeState.kExtended);
                // m_intake.takeIn(RobotMap.IntakeConstants.ROLLER_SPEED);
                // m_intake.setMagazineSpeed(RobotMap.IntakeConstants.MAGAZINE_SPEED);
                if(driveToTarget(0.7, RobotMap.AutonConstants.FOUR_BALL_STEP_FOURTEEN_TARGET_DISTANCE)){
                    m_drivetrain.periodic(0,0);
                    m_step = AutonStep.kStep14;
                    m_drivetrain.zeroEncoders();
                }
                else{
                    return;
                }
            }
            //step to retract intake if build finds a way to do this
            else if(m_step == AutonStep.kStep14){
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.INTAKE_WAITING_LOOPS){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStep15;
                }
            }
            //step to aim with the turret and launch
            else if (m_step == AutonStep.kStep15){
                m_launcher.targetAndLaunch();
                m_intakeInLoopCount ++;
                if(m_intakeInLoopCount >= RobotMap.AutonConstants.LOOPS_AFTER_LAUNCH){
                    //zeros the loop count to be used again
                    m_intakeInLoopCount = 0;
                    m_step = AutonStep.kStop;
                }
            }
            //step to finish zeroing everything for the end of auton
            else if(m_step == AutonStep.kStop){
                m_launcher.setFeederSpeed(0);
                m_launcher.setTurretSpeed(0);
                m_launcher.setFlywheelSpeed(0);
                m_intake.setRollerSpeed(0);
                m_intake.setMagazineSpeed(0);
                m_limelightVision.disableLEDs();
                m_intake.setIntakeExtension(IntakeState.kRetracted);
                if(m_doSysOut == true){
                    System.out.println("Auton Completed");
                }
                m_drivetrain.periodic(0, 0);
            }
        }
    }


    /**
     * Method for driving forward a specified amount of inches at a specified
     * @param speed speed robot will travel at 
     * @param target target distance in inches. Should always be positive. In order to turn the other way, pass in a negative speed
     * @return whether or not we have reached our target
     */
    public boolean driveToTarget(double speed, double target){
        //Assigns encoder values to variables to be used in the logic of this method
        double rightEncoder = m_drivetrain.getRightDriveEncoderPosition();
        double leftEncoder = m_drivetrain.getLeftDriveEncoderPosition();
        //Translates target in inches to target in encoder ticks
        target = target * RobotMap.AutonConstants.INCHES_TO_ENCODER_TICKS_LOWGEAR;

        if(speed < 0){
            target = target * -1;
            if(target < 0){
                if(leftEncoder > target || rightEncoder > target){
                    m_drivetrain.periodic(speed,0);
                    return false;
                }
                else{
                    m_drivetrain.periodic(0,0);
                    m_drivetrain.zeroEncoders();
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
        }
        else if(speed > 0){
            if((target > 0) && (speed > 0)){
                if((leftEncoder < target) || (rightEncoder < target)){
                    m_drivetrain.periodic(speed, 0);
                    return false;
                }
                else{
                    m_drivetrain.periodic(0,0);
                    m_drivetrain.zeroEncoders();
                    return true;
                }
            }
            else{
                System.out.println("Robot will never reach target; exiting pathing.");
                m_step = AutonStep.kStop;
                return false;
            }
        }
        else{
            System.out.println("Robot will never reach target; exiting pathing.");
            m_step = AutonStep.kStop;
            return false;
        }
    }

    /**
     * 
     * @param speed speed at which we want to turn
     * @param target target angle in degrees we want to turn to
     * @return whether or not we have reached our target angle
     */
    public boolean turnToAngle(double speed, double target){
        double currentAngle = m_drivetrain.getGyro();
        //if target is on the left, this if statement will run
        if(speed < 0){
            target = target * -1;
            //Rotate Bound is a percentage (we are within a certain percentage of are target angle where we will be close enough)
            if(currentAngle < (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.periodic(0,0);
                m_drivetrain.zeroGyro();
                m_drivetrain.zeroEncoders();
                if(m_doSysOut == true){
                    System.out.println("Current Angle: " + currentAngle + "Target Angle: " + (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND)));
                }
                return true;
            }
            else{
                m_drivetrain.periodic(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle + "Target Angle: " + (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND)));
                }
                return false;
            }
        }
        //if target is on the right, this if statement will run
        else{
            //Rotate Bound is a percentage (we are within a certain percentage of are target angle where we will be close enough)
            if(currentAngle > (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND))){
                m_drivetrain.periodic(0, 0);
                m_drivetrain.zeroEncoders();
                m_drivetrain.zeroGyro();
                if(m_doSysOut == true){
                    System.out.println("Current Angle: " + currentAngle + "Target Angle: " + (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND)));
                }
                return true;
            }
            else{
                m_drivetrain.periodic(0, speed);
                if(m_doSysOut == true){
                    System.out.println("Not At Target Angle " + currentAngle + "Target Angle: " + (target * (1 - RobotMap.AutonConstants.ROTATE_BOUND)));
                }
                return false;
            }
        }
    }

    //Method to set the auton path from the shuffleboard. 0 = Right Line, 1 = Right Wall, 2 = Left Wall
    private void selectPath(){
        if(m_currentAutonPath == 2){
            System.out.println("Setting Auton to Right Line Path");
            m_path = AutonPath.kTwoBall;
        }
        else if(m_currentAutonPath == 3){
            System.out.println("Setting Auton to Right Wall Path");
            m_path = AutonPath.kThreeBall;
        }
        else{
            m_currentAutonPath = 2;
            System.out.println("Setting Auton to Right Line Path");
            m_path = AutonPath.kTwoBall;
        }
    }
}