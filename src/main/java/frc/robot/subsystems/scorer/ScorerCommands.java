package frc.robot.subsystems.scorer;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ScorerCommands {
    public static Command CS_setRunning(Scorer m_Scorer, boolean runScorer) {
        return Commands.run(() -> m_Scorer.CS_setRunning(runScorer));
    }

    public static Command CS_reverseScorer(Scorer m_Scorer, boolean reverse) {
        return Commands.runOnce(() -> m_Scorer.CS_reverseScorer(reverse));
    }

    public static Command CS_runSetpoint(Scorer m_Scorer, Angle angle) {
        return Commands.runOnce(() -> m_Scorer.CS_runSetpoint(angle));
    }

}
