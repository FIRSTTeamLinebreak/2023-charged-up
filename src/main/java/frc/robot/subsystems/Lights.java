package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem that handles the turntable mechanism. */
public class Lights extends SubsystemBase {
    // Define the singleton
    private static Lights instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static Lights getInstance() {
        if (instance == null) {
            instance = new Lights();
        }

        return instance;
    }

    private final PWM redLight;
    private int redVal;
    private final PWM greenLight;
    private int greenVal;
    private final PWM blueLight;
    private int blueVal;

    /** Initializes a new Turntable subsystem object. */
    private Lights() {
        redLight = new PWM(0);
        greenLight = new PWM(1);
        blueLight = new PWM(2);
        redLight.setBounds(0.00000000001, 0.0000000000001, 100, 0.1, 100);
        blueLight.setDisabled();
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        redLight.setRaw(redVal);
        greenLight.setRaw(greenVal);
        blueLight.setRaw(blueVal);
    }

    public int getRedVal() {
        return redVal;
    }

    public void setRedVal(int redVal) {
        this.redVal = redVal;
    }

    public int getGreenVal(){
        return greenVal;
    }

    public void setGreenVal(int greenVal){
        this.greenVal = greenVal;
    }

    public int getBlueVal() {
        return blueVal;
    }

    public void setBlueVal(int blueVal){
        this.blueVal = blueVal;
    }
}
