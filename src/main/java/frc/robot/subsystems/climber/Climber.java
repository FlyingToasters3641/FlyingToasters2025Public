package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Angle;
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
            this.io.CL_Stop();
        } else {
            this.io.CL_runSetpoint(this.CL_setpoint);
        }


        CL_actual.updateClimberRotation(this.inputs.rotation);
        CL_target.updateClimberRotation(this.inputs.setpointRotation);
        CL_goal.updateClimberRotation(this.CL_setpoint);

    }

    public void CL_setRotation(double rotation){
        io.CL_setRotation(rotation);
    }

    public Angle CL_getRotation() {
        return io.CL_getRotation();
    }

    public void CL_Stop(){
        io.CL_Stop();
    }

    public void CL_setServo(double angle){
        io.CL_setServo(angle);
    }

    public void CL_Home(){
        io.CL_Home();
    }

    public void CL_setPosition(double position){
        io.CL_setPosition(position);
    }   
    
}
