package frc.robot;

public class Util {
    public static Double applyDeadzone(double deadzone, double input) {
        return Math.abs(input) > deadzone ? input : 0.0;
    }
}
