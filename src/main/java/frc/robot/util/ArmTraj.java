package frc.robot.util;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class ArmTraj {
    private final String name;
    private final double[] theta;
    private final double[] phi;
    private final double[] h;
    private final double[] x;
    private final double[] y;

    public int length;

    // Constructor
    public ArmTraj(String name, double[] theta, double[] phi, double[] h, double[] x, double[] y) {
        this.name = name;
        this.theta = theta;
        this.phi = phi;
        this.h = h;
        this.x = x;
        this.y = y;

        // gereksiz
        if (this.theta.length == this.phi.length && this.phi.length == this.h.length) {
        this.length = this.theta.length;
        } else {this.length = 99;}
    }

    // Getters (no setters since x and y are immutable)
    public String getName() {
        return name;
    }

    public double[] getTheta() {
        return theta;
    }

    public double[] getPhi() {
        return phi;
    }

    public double[] getH() {
        return h;
    }

    public double[] getX() {
        return x;
    }

    public double[] getY() {
        return y;
    }

    @Override
    public String toString() {
        return "ArmTraj{name='" + name + "', theta=" + java.util.Arrays.toString(theta) + ", h=" + java.util.Arrays.toString(h) + '}';
    }
}
