package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorVisualizer measuredVisualizer;
    private final ElevatorVisualizer goalVisualizer;



    private Distance setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io){
        this.io = io;

        measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        measuredVisualizer.update(this.inputs.EL_position);
        goalVisualizer.update(this.setpoint);

    }

    public void setELPosition(double position){
        io.setELPosition(position);
    }


}
