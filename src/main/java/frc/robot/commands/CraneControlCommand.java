package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Crane;
import java.util.function.Supplier;

/** The Command used to control the crane. */
public class CraneControlCommand extends CommandBase {
    private final Crane craneSub;

    private final Supplier<Double> pivotPositionSupplier;
    private final Supplier<Double> armPositionSupplier;
    private final Supplier<Double> clawSpeedSupplier;

    private final PIDController pivotPidController;
    private final PIDController armPidController;

    /** Creates the Command used for teleop control of the robot arm. */
    public CraneControlCommand(Supplier<Double> pivotPositionSupplier, Supplier<Double> armPositionSupplier, Supplier<Double> clawSpeedSupplier) {
        this.pivotPositionSupplier = pivotPositionSupplier;
        this.armPositionSupplier = armPositionSupplier;
        this.clawSpeedSupplier = clawSpeedSupplier;

        pivotPidController = new PIDController(0.05, 0.0, 0.0);
        armPidController = new PIDController(0.05, 0.0, 0.0);

        craneSub = Crane.getInstance();
        addRequirements(craneSub);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        craneSub.setPivotSpeed(pivotPidController.calculate(craneSub.getPivotPosition(), pivotPositionSupplier.get()) * 0.05);
        craneSub.setArmSpeed(armPidController.calculate(craneSub.getArmPosition(), armPositionSupplier.get()) * 0.05);
        craneSub.setClawSpeed(clawSpeedSupplier.get());
    }

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        craneSub.stop();
    }

    /** Whether the command has finished. If true, calls end().
     *
     * @return Weather this command is done.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
