package frc.robot.util.custom;



/**
 * PID constants used to create PID controllers
 * This class is special becuase of its overloaded constructors
 * It just helps keep everything organized
 */
public class GainConstants {

    
    
    // Generic
    private double P; 
    private double I;
    private double D;

    // Spark
    private double FF;
    private double iZone;
    private double minOutput;
    private double maxOutput;

    // Talon
    private double A;
    private double S;
    private double V;
    private double G;

    // Generic
    public GainConstants() {
        this(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    }

    // Generic
    public GainConstants(double P, double I, double D) {
        this(P, I, D, 0);
    }

    // Spark
    public GainConstants(double P, double I, double D, double FF) {
        this(P, I, D, FF, Double.POSITIVE_INFINITY, -1, 1, 0d, 0d, 0d,0d);
    }
    
    // Spark
    public GainConstants(double P, double I, double D, double minOutput, double maxOutput) {
        this(P, I, D, 0d, Double.POSITIVE_INFINITY, minOutput, maxOutput, 0d, 0d, 0d,0d);
    }

    public GainConstants(double P, double I, double D, double A, double S, double V, double G) {
        this(P, I, D, 0d, 0d, 0d, 0d, A, S, V, G);
    }

    // Talon
    public GainConstants(double P, double I, double D, double S, double V, double G) {
        this(P, I, D, 0d, S, V, G);
    }

    // Generic
    public GainConstants(double P, double I, double D, double FF, double iZone, double minOutput, double maxOutput, double A, double S, double V, double G) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.FF = FF;
        this.iZone = iZone;
        this.minOutput = minOutput;
        this.maxOutput = maxOutput;
        this.A = A;
        this.S = S;
        this.V = V;
        this.G = G;
    }

    public double getP() {
        return P;
    }

    public double getI() {
        return I;
    }

    public double getD() {
        return D;
    }

    public double getFF() {
        return FF;
    }

    public double getIZone() {
        return iZone;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public double getA() {
        return A;
    }

    public double getS() {
        return S;
    }

    public double getV() {
        return V;
    }

    public double getG() {
        return G;
    }

    public GainConstants withP(double P) {
        this.P = P;
        return this;
    }

    public GainConstants withI(double I) {
        this.I = I;
        return this;
    }

    public GainConstants withD(double D) {
        this.D = D;
        return this;
    }

    public GainConstants withS(double S) {
        this.S = S;
        return this;
    }

    public GainConstants withA(double A) {
        this.A = A;
        return this;
    }

    public GainConstants withV(double V) {
        this.V = V;
        return this;
    }

    public GainConstants withG(double G) {
        this.G = G;
        return this;
    }

    public GainConstants withGains(double P, double I, double D, double A, double S, double V, double G) {
        this.P = P;
        this.I = I;
        this.D = D;
        this.A = A;
        this.S = S;
        this.V = V;
        this.G = G;
        return this;
    }

    public GainConstants withGains(double P, double I, double D, double S, double V, double G) {
        return withGains(P, I, D, 0, S, V, G);
    }

    public GainConstants withPID(double P, double I, double D) {
        this.P = P;
        this.I = I;
        this.D = D;
        return this;
    }

}