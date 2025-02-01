package frc.robot.subsystems.scorer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase{
    private ScorerIO io;
    private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();
    private final ScorerVisualizer CS_measuredVisualizer;
    private final ScorerStates CS_actual;

    public static double frontTalonVoltage;

    public Scorer(ScorerIO io) {
        this.io = io;
        this.CS_actual = ScorerStates.getCS_measuredInstance();
        CS_measuredVisualizer = new ScorerVisualizer("Measured", Color.kBlack);

        CS_actual.updateScorerAngle(this.inputs.CS_angle);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Scorer", inputs); // Log telemetry
        CS_measuredVisualizer.update(this.inputs.CS_angle);
    }

    public void CS_setRoller(double speed) {
        io.CS_setRoller(speed);
    }

    public void CS_reverseScorer(boolean reverse) {
        io.CS_reverseScorer(reverse);
    }

    public void CS_runSetpoint(Angle angle) {
        io.CS_runSetpoint(angle);
    }

    public Angle getCSAngle() {
        return(this.inputs.CS_angle);
    }

   
}
