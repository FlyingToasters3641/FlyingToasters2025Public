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

    private final ElevatorVisualizer EL_measuredVisualizer;
    private final ElevatorVisualizer EL_goalVisualizer;




    private Distance setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io){
        this.io = io;

        EL_measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        EL_goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        EL_measuredVisualizer.update(this.inputs.EL_position);
        EL_goalVisualizer.update(this.setpoint);

    }

    public void setELPosition(double position){
        io.setELPosition(position);
    }

    public void setGoal(double goal){
        io.setELGoal(goal);
    }

    public void applyELVolts(){
        io.applyELVolts();
    }

}
