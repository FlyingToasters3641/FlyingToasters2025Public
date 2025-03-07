package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public class Scorer extends SubsystemBase{
    private ScorerIO io;
    private final ScorerIOInputsAutoLogged inputs = new ScorerIOInputsAutoLogged();
    private final ScorerVisualizer CS_measuredVisualizer;
    private Angle setpoint = Degrees.of(0.0);
    private AbstractDriveTrainSimulation driveSimulation;

    // Constructor for SIM Scorer
    public Scorer(ScorerIO io, AbstractDriveTrainSimulation driveSimulation) {
        this.io = io;
        this.io.CS_setPID(0.5, 0.0, 0.0);
        this.CS_measuredVisualizer = new ScorerVisualizer("Measured", Color.kBlack);

        this.driveSimulation = driveSimulation;      
    }
    
    // Constructor for REAL Scorer
    public Scorer(ScorerIO io) {
        this.io = io;
        this.io.CS_setPID(0.5, 0.0, 0.0);
        this.CS_measuredVisualizer = new ScorerVisualizer("Measured", Color.kBlack);

        this.driveSimulation = driveSimulation;      
    }

    @Override
    public void periodic() {
        super.periodic();
        this.io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Scorer", inputs); // Log telemetry
        this.io.CS_runSetpoint(this.setpoint);
        this.CS_measuredVisualizer.update(this.inputs.CS_angle);
    }

    public void CS_setRoller(double speed) {
        io.CS_setRoller(speed);
    }

    public void CS_reverseScorer(boolean reverse) {
        io.CS_reverseScorer(reverse);
    }

    public Command CS_runSetpoint(Angle angle) {
        return runOnce(() -> this.setpoint = angle);
    }

    public Angle CS_getAngle() {
        return(this.inputs.CS_angle);
    }

    public boolean CS_getAlgae() {
        return(this.inputs.CS_algae);
    }

    public boolean CS_getCoral() {
        return(this.inputs.CS_coral);
    }

    public void CS_setSpeed(double speed){
        io.CS_setSpeed(speed);
    }
   
    public void CS_shootSimAlgae(){
        io.CS_shootSimAlgae(driveSimulation);
    }

    public void CS_shootL1SimCoral(){
        io.CS_shootL1SimCoral(driveSimulation);
    }

    public void CS_shootL2SimCoral(){
        io.CS_shootL2SimCoral(driveSimulation);
    }

    public void CS_shootL3SimCoral(){
        io.CS_shootL3SimCoral(driveSimulation);
    }

    public void CS_shootL4SimCoral(){
        io.CS_shootL4SimCoral(driveSimulation);
    }
}
