package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    
    private ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ElevatorVisualizer EL_measuredVisualizer;
    private final ElevatorVisualizer EL_goalVisualizer;




    public Distance EL_setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io){
        this.io = io;
        this.io.EL_setPID(0.3, 0, 0);

        //Goal shows where the elevator wants to be. Measures shows where the Elevator is currently.
        EL_measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        EL_goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    @Override
    public void periodic() {
        super.periodic();

        
        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        //Updates all positions in ElevatorStates and for the simulation
        EL_measuredVisualizer.update(this.inputs.position);
        EL_goalVisualizer.update(this.EL_setpoint);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.ELStop();
        } else {
            this.io.EL_runSetpoint(this.EL_setpoint);
        }
    }


    public void setELPosition(double position){
        io.setELPosition(position);
    }

    public Distance getELPosition(){
        return io.EL_getPosition();
    }

    public void ELStop(){
        io.ELStop();
    }

    public void EL_runVolts(double volts){
        io.EL_runVolts(Volts.of(volts));
    }

    public void EL_setSpeed(double speed){
        io.EL_setSpeed(speed);
    }
    

}
