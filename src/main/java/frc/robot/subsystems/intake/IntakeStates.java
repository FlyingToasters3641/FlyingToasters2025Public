package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.RobotState;

import static edu.wpi.first.units.Units.*;

public class IntakeStates {


    //Measures to feed into the Visualizer.
    private static IntakeStates IN_measuredInstance;
    

    private MutAngle IN_angle;

    private IntakeStates() {
        IN_angle = Radians.mutable(0);
    }

    public static IntakeStates getIN_measuredInstance() {
        if (IN_measuredInstance == null) {
            IN_measuredInstance = new IntakeStates();
        }
        return IN_measuredInstance;
    }

    public static IntakeStates getIN_desiredInstance() {
        if (IN_measuredInstance == null) {
            IN_measuredInstance = new IntakeStates();
        }
        return IN_measuredInstance;
    }

    public static IntakeStates getIN_goalInstance() {
        if (IN_measuredInstance == null) {
            IN_measuredInstance = new IntakeStates();
        }
        return IN_measuredInstance;
    }

    public MutAngle getIN_position() {
        return IN_angle;
    }

    public void updateIntakePosition(Angle position) {
        IN_angle.mut_replace(position);
    }

    
}
