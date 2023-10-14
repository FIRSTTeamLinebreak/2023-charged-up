package frc.robot;

import edu.wpi.first.math.Pair;

/** Some utility functions. */
public class Util {
    /**
     * Applies a deadzone over a linear space.
     *
     * @param deadzone The zone centered around zero with range of [-deadzone, deadzone]
     * @param input The input to apply a deadzone too
     *
     * @return returns a value of either 0 or the input
     */
    public static Double applyLinearDeadzone(double deadzone, double input) {
        return Math.abs(input) > deadzone ? input : 0.0;
    }

    /**
     * Applies a deadzone over a circular space.
     *
     * @param deadzone The zone centered around zero with a radius of deadzone
     * @param x the input on the x axis
     * @param y the input on the y axis
     *
     * @return an array of size two of form [x, y] where either both x and y are zero or they both reflect the input
     */
    public static Pair<Double, Double> applyCircularDeadzone(double deadzone, double x, double y) {
        Double magnitude = Math.sqrt((x * x) + (y * y));
        Pair<Double, Double> ret  = new Pair<Double, Double>(x, y);
        Pair<Double, Double> zero = new Pair<Double, Double>(0.0, 0.0);

        return magnitude > deadzone ? ret : zero;
    }
}
