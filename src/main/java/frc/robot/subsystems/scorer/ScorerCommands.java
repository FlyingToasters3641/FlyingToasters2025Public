package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ScorerCommands {
    public static Command CS_setRunning(Scorer m_Scorer, double speed) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(speed));
    }

    public static Command CS_reverseScorer(Scorer m_Scorer, boolean reverse) {
        return Commands.runOnce(() -> m_Scorer.CS_reverseScorer(reverse));
    }

    public static Command CS_runSetpoint(Scorer m_Scorer, Angle angle) {
        return m_Scorer.CS_runSetpoint(angle);
    }

    public static Command CS_scoreCoral(Scorer m_Scorer) {
        return Commands.run(() -> m_Scorer.CS_setRoller(0.75)).until(() -> m_Scorer.CS_getCoral()).andThen(Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0)));
    }

    public static Command CS_goToRest(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(0));
    }

    public static Command CS_net(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(160));
    }

    public static Command CS_removeAlgae(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(-1.0)).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0)));
    }

    public static Command CS_goToL4(Scorer m_Scorer) {    
        return m_Scorer.CS_runSetpoint(Degrees.of(200));
    }

    public static Command CS_intakeAlgae(Scorer m_Scorer) {
        return CS_setRunning(m_Scorer, -1.0);
    }

    public static Command CS_intakeCoral(Scorer m_Scorer) {
        return CS_setRunning(m_Scorer, 1.0);
    }

    public static Command CS_stop(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0));
    }

    public static Command CS_goToAlgae(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(200));
    }

}
