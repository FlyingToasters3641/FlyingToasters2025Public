package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeVisualizer IN_measuredVisualizer;
    private Angle setpoint = Degrees.of(0.0);

    public Intake(IntakeIO io) {
        this.io = io;
        this.io.IN_setPID(8.0, 0.0, 0.0);
        this.IN_measuredVisualizer = new IntakeVisualizer("Measured", Color.kBlack);
    }

    @Override
    public void periodic() {
        super.periodic();
        io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Intake", inputs); // Log telemetry
        this.io.IN_runSetpoint(this.setpoint);
        this.IN_measuredVisualizer.update(this.inputs.IN_angle);
    }

    public void IN_setRunning(boolean runIntake) {
        io.IN_setRunning(runIntake);
    }

    public void IN_reverseIntake(double speed, boolean reverse) {
        io.IN_reverseIntake(reverse, speed);
    }

    public Command IN_runSetpoint(Angle angle) {
        return runOnce(() -> this.setpoint = angle);
    }

    
    public void IN_setRoller(double speed) {
        io.IN_setRoller(speed);
    }

    public Angle IN_getAngle() {
        return(this.inputs.IN_angle);
    }

}
