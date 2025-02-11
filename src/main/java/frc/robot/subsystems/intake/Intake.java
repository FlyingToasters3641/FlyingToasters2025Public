package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
    private final IntakeVisualizer IN_measuredVisualizer;

    private final IntakeStates IN_actual;
    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();
    public static double frontTalonVoltage;

    public Angle IN_setangle = Radians.of(0.0);

    public Intake(IntakeIO io) {
        this.io = io;
        this.IN_actual = IntakeStates.getIN_measuredInstance();
        IN_measuredVisualizer = new IntakeVisualizer("Measured", Color.kBlack);
        IN_actual.updateIntakePosition(this.inputs.IN_angle);

    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Intake", inputs); // Log telemetry
        IN_measuredVisualizer.update(this.inputs.IN_angle);

    }

    public void IN_setRunning(boolean runIntake) {
        io.IN_setRunning(runIntake);
    }

    public void IN_reverseIntake(double speed, boolean reverse) {
        io.IN_reverseIntake(reverse, speed);
    }

    public void IN_runSetpoint(Angle angle) {
        io.IN_runSetpoint(angle);
    }

    
    public void IN_setRoller(double speed) {
        io.IN_setRoller(speed);
    }
    public Angle getINAngle() {
        return(this.inputs.IN_angle);
    }

}
