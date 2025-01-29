package frc.robot.subsystems.scorer;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase{
    private ScorerIO io;
    private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();

    public static double frontTalonVoltage;

    public Scorer(ScorerIO io) {
        this.io = io;
     
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Scorer", inputs); // Log telemetry
    }

    public void CS_setRunning(boolean runScorer) {
        io.CS_setRunning(runScorer);
    }

    public void CS_reverseScorer(boolean reverse) {
        io.CS_reverseScorer(reverse);
    }

   
}
