package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScorerCommands {
    public static Command CS_setRunning(Scorer m_Scorer, double speed) {
        return Commands.run(() -> m_Scorer.CS_setRoller(speed));
    }

    public static Command CS_reverseScorer(Scorer m_Scorer, boolean reverse) {
        return Commands.runOnce(() -> m_Scorer.CS_reverseScorer(reverse));
    }

    public static Command CS_runSetpoint(Scorer m_Scorer, Angle angle) {
        return m_Scorer.CS_runSetpoint(angle);
    }

    public static Command CS_goToL1(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(90));
    }

    public static Command CS_goToRest(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(0));
    }

    public static Command CS_net(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(40));
    }

}
