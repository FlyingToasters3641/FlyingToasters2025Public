package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class ScorerStates {
    private static ScorerStates CS_measuredInstance;

    private MutAngle CS_angle;

    private ScorerStates() {
        CS_angle = Radians.mutable(0);
    }

    public static ScorerStates getCS_measuredInstance() {
        if (CS_measuredInstance == null) {
            CS_measuredInstance = new ScorerStates();
        }
        return CS_measuredInstance;
    }

    public static ScorerStates getCS_desiredInstance() {
        if (CS_measuredInstance == null) {
            CS_measuredInstance = new ScorerStates();
        }
        return CS_measuredInstance;
    }

    public MutAngle getCS_angle() {
        return CS_angle;
    }

    public void updateScorerAngle(Angle angle) {
        CS_angle.mut_replace(angle);
    }
}
