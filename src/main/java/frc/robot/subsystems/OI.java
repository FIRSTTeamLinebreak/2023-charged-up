package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OI extends SubsystemBase {
    // Define the singleton
    private static OI instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }

        return instance;
    }

    /** Initializes a new Turntable subsystem object. */
    private OI() {

    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {

    }

}
