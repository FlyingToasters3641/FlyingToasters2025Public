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
    public Angle CL_setpoint = new Rotation2d(0, 0).getMeasure();
    public double CL_position = 0.0;
    public double CL_currentVelocity = 0.0;


    public Climber(ClimberIO io) {
        this.io = io;
        this.io.CL_setPID(0.3, 0, 0);
        CL_measuredVisualizer = new ClimberVisualizer("Measured");
        CL_goalVisualizer = new ClimberVisualizer("Goal");
    }

    @Override
    public void periodic() {
        super.periodic();

        
        this.io.updateInputs(inputs);
        CL_currentVelocity = inputs.CL_currentVelocity;
        Logger.processInputs("Climber", inputs);

        //Updates all positions in ElevatorStates and for the simulation
        CL_measuredVisualizer.update(this.inputs.rotation);
        CL_goalVisualizer.update(this.CL_setpoint);

        if(edu.wpi.first.wpilibj.RobotState.isDisabled()) {
            this.io.CL_Stop();
        } else {
            this.io.CL_runSetpoint(this.CL_setpoint);
        }

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

    public void CL_setSpeed(double speed){
        io.CL_setSpeed(speed);
    } 

    public boolean CL_getExtended() {
        return io.CL_getExtended();
    }   

    public boolean CL_getServoDisengaged() {
        return io.CL_getServoDisengaged();
    }
    
}
