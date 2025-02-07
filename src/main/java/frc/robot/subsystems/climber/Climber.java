package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.elevator.ElevatorVisualizer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.geometry.Rotation2d;

public class Climber extends SubsystemBase{
    private ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private final ClimberVisualizer CL_measuredVisualizer;
    private final ClimberVisualizer CL_goalVisualizer;

    private final ClimberStates CL_actual;
    private final ClimberStates CL_target;
    private final ClimberStates CL_goal;

    public Angle CL_setpoint = new Rotation2d(0, 0).getMeasure();


    public Climber(ClimberIO io) {
        this.io = io;
        this.io.CL_setPID(0.3, 0, 0);
        this.CL_actual = ClimberStates.getCL_measuredInstance();
        this.CL_target = ClimberStates.getCL_desiredInstance();
        this.CL_goal = ClimberStates.getCL_goalInstance();

        CL_measuredVisualizer = new ClimberVisualizer("Measured");
        CL_goalVisualizer = new ClimberVisualizer("Goal");
    }

    @Override
    public void periodic() {
        super.periodic();

        
        this.io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);

        //Updates all positions in ElevatorStates and for the simulation
        CL_measuredVisualizer.update(this.inputs.rotation);
        CL_goalVisualizer.update(this.CL_setpoint);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.CLStop();
        } else {
            this.io.CL_runSetpoint(this.CL_setpoint);
        }


        CL_actual.updateClimberRotation(this.inputs.rotation);
        CL_target.updateClimberRotation(this.inputs.setpointRotation);
        CL_goal.updateClimberRotation(this.CL_setpoint);

    }

    public void setCLRotation(double rotation){
        io.setCLRotation(rotation);
    }

    public Angle getCLRotation() {
        return io.getCLrotation();
    }

    public void CLStop(){
        io.CLStop();
    }
    
}
