package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase{
    private IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    IntakeIOTalonFX IntakeTalonFX = new IntakeIOTalonFX();
    public static double frontTalonVoltage;

    public Intake(IntakeIO io) {
        this.io = io;
     
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs); // Update inputs from IO
        Logger.processInputs("Intake", inputs); // Log telemetry
    }

    
   
}
