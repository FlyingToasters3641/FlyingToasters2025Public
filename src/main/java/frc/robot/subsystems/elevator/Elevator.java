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

    private final ElevatorStates actual;
    private final ElevatorStates target;
    private final ElevatorStates goal;




    public Distance setpoint = Inches.of(0.0);

    public Elevator(ElevatorIO io){
        this.io = io;
        this.io.EL_setPID(0.3, 0, 0);
        this.actual = ElevatorStates.getEL_measuredInstance();
        this.target = ElevatorStates.getEL_desiredInstance();
        this.goal = ElevatorStates.getEL_goalInstance();


        EL_measuredVisualizer = new ElevatorVisualizer("Measured", Color.kBlack);
        EL_goalVisualizer = new ElevatorVisualizer("Goal", Color.kBlue);
    }

    @Override
    public void periodic() {
        super.periodic();

        this.io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        EL_measuredVisualizer.update(this.inputs.position);
        EL_goalVisualizer.update(this.setpoint);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.ELStop();
        } else {
            this.io.EL_runSetpoint(this.setpoint);
        }


        actual.updateElevatorPosition(this.inputs.position);
        target.updateElevatorPosition(this.inputs.setpointPosition);
        goal.updateElevatorPosition(this.setpoint);

    }

    public void setELPosition(double position){
        io.setELPosition(position);
    }

    // public void setGoal(Distance goal){
    //     io.setELGoal(goal);
    // }

    // public void applyELVolts(){
    //     io.applyELVolts();
    // }
    

}
