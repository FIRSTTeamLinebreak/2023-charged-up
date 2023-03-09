package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem that handles the turntable mechanism. */
public class Turntable extends SubsystemBase {
    // Define the singleton
    private static Turntable instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static Turntable getInstance() {
        if (instance == null) {
            instance = new Turntable();
        }

        return instance;
    }

    private final CANSparkMax motor;
    private final double targetSpeed = 0.5; // Speed of turntable [-1, 1]. @TODO: Tune

    private boolean userControl = false; // Weather the turntable is controlled via the user
    private double userControlSpeed = 0.0; // Speed of the turntable [-1, 1], while the turntable is under user control

    /** Initializes a new Turntable subsystem object. */
    private Turntable() {
        motor = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        motor.set(userControl ? userControlSpeed : targetSpeed);
    }

    /** Toggles weather the turntable is controlled via the user. */
    public void toggleUserControl() {
        userControl = !userControl;
        SmartDashboard.putBoolean("User Controlled Turntable", userControl);
    }

    /** Sets the desired speed for the turntable while under user control
     * @param speed The desired speed [-1, 1]
     */
    public void setUserControlSpeed(double speed) {
        userControlSpeed = speed;
    }
}
