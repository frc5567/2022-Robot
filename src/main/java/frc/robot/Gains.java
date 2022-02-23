package frc.robot;
// Note: When tuning, tune in the order of PDI

/**
 * Class for holding PID constants as well as gains in robot map
 */
public class Gains {
    //Declares the variables to be passed into the gains function
    public final double kP;
    public final double kI;
    public final double kD;
    public final double kF;
    public final int kIzone;
    public final double kPeakOutput;

    /**
     * Contructor for Gains objects
     * These objects store PID constants for easier reading in the RobotMap
     * @param _kP the proportionality constant. How much to change the value/power by
     * @param _kI the integral constant. Error between the current and desired value. Bumps the value/power to actually hit the target
     * @param _kD the derivative constant. The rate of change. It helps to dampen oscillation
     * @param _kF the feed-forward constant. The minimum amount of power needed to overcome inertia (to actually get us to move). Use characterization to find this
     * @param _kIzone the intergral zone -> if absolute closed loop error exceeds this value, error is reset
     * @param _kPeakOutput the peak output of the PID control
     */
    public Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput){
        kP = _kP;
        kI = _kI;
        kD = _kD;
        kF = _kF;
        kIzone = _kIzone;
        kPeakOutput = _kPeakOutput;
    }
}
