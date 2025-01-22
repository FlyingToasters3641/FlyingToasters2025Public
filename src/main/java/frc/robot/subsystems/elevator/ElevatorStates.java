package frc.robot.subsystems.elevator;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.RobotState;

import static edu.wpi.first.units.Units.*;

public class ElevatorStates {


    //Measures to feed into the Visualizer.
    private static ElevatorStates EL_measuredInstance;
    private static ElevatorStates EL_desiredInstance;
    private static ElevatorStates EL_goalInstance;

    private MutDistance EL_position;

    private ElevatorStates() {
        EL_position = Inches.mutable(0);
    }

    public static ElevatorStates getEL_measuredInstance() {
        if (EL_measuredInstance == null) {
            EL_measuredInstance = new ElevatorStates();
        }
        return EL_measuredInstance;
    }

    public static ElevatorStates getEL_desiredInstance() {
        if (EL_desiredInstance == null) {
            EL_desiredInstance = new ElevatorStates();
        }
        return EL_desiredInstance;
    }

    public static ElevatorStates getEL_goalInstance() {
        if (EL_goalInstance == null) {
            EL_goalInstance = new ElevatorStates();
        }
        return EL_goalInstance;
    }

    public Distance getEL_position() {
        return EL_position;
    }

    public void updateElevatorPosition(Distance position) {
        EL_position.mut_replace(position);
    }

    
}
