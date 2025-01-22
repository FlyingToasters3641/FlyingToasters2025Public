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

    private final ElevatorStates EL_actual;
    private final ElevatorStates EL_target;
    private final ElevatorStates EL_goal;




    public Distance EL_setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io){
        this.io = io;
        this.io.EL_setPID(0.3, 0, 0);
        this.EL_actual = ElevatorStates.getEL_measuredInstance();
        this.EL_target = ElevatorStates.getEL_desiredInstance();
        this.EL_goal = ElevatorStates.getEL_goalInstance();

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


        EL_actual.updateElevatorPosition(this.inputs.position);
        EL_target.updateElevatorPosition(this.inputs.setpointPosition);
        EL_goal.updateElevatorPosition(this.EL_setpoint);

    }


    public void setELPosition(double position){
        io.setELPosition(position);
    }

    public void ELStop(){
        io.ELStop();
    }
    

}
