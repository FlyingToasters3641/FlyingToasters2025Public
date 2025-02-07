package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class ClimberStates {
    //Measures to feed into the Visualizer.
    private static ClimberStates CL_measuredInstance;
    private static ClimberStates CL_desiredInstance;
    private static ClimberStates CL_goalInstance;

    private MutAngle CL_rotation;

    private ClimberStates() {
        CL_rotation = Radians.mutable(0);
    }

    public static ClimberStates getCL_measuredInstance() {
        if (CL_measuredInstance == null) {
            CL_measuredInstance = new ClimberStates();
        }
        return CL_measuredInstance;
    }

    public static ClimberStates getCL_desiredInstance() {
        if (CL_desiredInstance == null) {
            CL_desiredInstance = new ClimberStates();
        }
        return CL_desiredInstance;
    }

    public static ClimberStates getCL_goalInstance() {
        if (CL_goalInstance == null) {
            CL_goalInstance = new ClimberStates();
        }
        return CL_goalInstance;
    }

    public Angle getCL_rotation() {
        return CL_rotation;
    }

    public void updateClimberRotation(Angle rotation) {
        CL_rotation.mut_replace(rotation);
    }
}
