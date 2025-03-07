package frc.robot.subsystems.scorer;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.DoubleSupplier;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.BehaviorTree.Blackboard;

public class ScorerCommands {
    public static Command CS_setRunning(Scorer m_Scorer, DoubleSupplier speed) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(speed.getAsDouble()));
    }

    public static Command CS_setConstantRunning(Scorer m_Scorer, DoubleSupplier speed) {
        return Commands.run(() -> m_Scorer.CS_setRoller(speed.getAsDouble()));
    }

    public static Command CS_reverseScorer(Scorer m_Scorer, boolean reverse) {
        return Commands.runOnce(() -> m_Scorer.CS_reverseScorer(reverse));
    }

    public static Command CS_runSetpoint(Scorer m_Scorer, Angle angle) {
        return m_Scorer.CS_runSetpoint(angle);
    }

    public static Command CS_runAlgaePosition(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(ScorerConstants.CS_INTAKE_ALGAE);
    }

    public static Command CS_scoreCoral(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(-0.75)).until(() -> m_Scorer.CS_getCoral() == false).andThen(Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0)));
    }

    public static Command CS_scoreCoralL4(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(0.75)).until(() -> m_Scorer.CS_getCoral() == false).andThen(new WaitCommand(0.5)).andThen(Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0)));
    }

    public static Command CS_goToRest(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(0));
    }

    public static Command CS_net(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(170));
    }

    public static Command CS_removeAlgae(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(-0.6)).andThen(new WaitCommand(1.0)).andThen(Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0)));
    }

    public static Command CS_goToL4(Scorer m_Scorer) {    
        return m_Scorer.CS_runSetpoint(Degrees.of(ScorerConstants.CS_L4_ANGLE));
    }

    public static Command CS_intakeAlgae(Scorer m_Scorer) {
        return CS_setRunning(m_Scorer, () -> 0.6);
    }

    public static Command CS_intakeCoral(Scorer m_Scorer) {
        return CS_setConstantRunning(m_Scorer, () -> -0.6).until(() -> !m_Scorer.CS_getCoral()).andThen(
            new SequentialCommandGroup(
                CS_setRunning(m_Scorer, () -> 0.1),
                new WaitCommand(0.2),
                CS_setRunning(m_Scorer, () -> 0.0)
            ));
    }

    public static Command CS_stop(Scorer m_Scorer) {
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(0.0));
    }

    public static Command CS_goToAlgae(Scorer m_Scorer) {
        return m_Scorer.CS_runSetpoint(Degrees.of(200));
    }

    public static Command CS_joystickControl(Scorer m_Scorer, DoubleSupplier axis){
        return Commands.run(() -> m_Scorer.CS_setSpeed(axis.getAsDouble()));
    }

    public static Command CS_shootSimAlgae(Scorer m_Scorer, AbstractDriveTrainSimulation driveSimulation){
        return Commands.runOnce(() -> m_Scorer.CS_shootSimAlgae());
    }

    public static Command CS_shootL1SimCoral(Scorer m_Scorer){
        return Commands.runOnce(() -> m_Scorer.CS_shootL1SimCoral());
    }

    public static Command CS_shootL2SimCoral(Scorer m_Scorer){
        return Commands.runOnce(() -> m_Scorer.CS_shootL2SimCoral());
    }

    public static Command CS_shootL3SimCoral(Scorer m_Scorer){
        return Commands.runOnce(() -> m_Scorer.CS_shootL3SimCoral());
    }

    public static Command CS_shootL4SimCoral(Scorer m_Scorer){
        return Commands.runOnce(() -> m_Scorer.CS_shootL4SimCoral());
    }

    public static Command CS_setPivotToBlackboard(Scorer m_Scorer, Blackboard blackboard){
        return Commands.runOnce(() -> m_Scorer.CS_runSetpoint(Degrees.of(blackboard.getTargetScorerAngle("target"))));
    }

    public static Command CS_setSpeedToBlackboard(Scorer m_Scorer, Blackboard blackboard){
        return Commands.runOnce(() -> m_Scorer.CS_setRoller(blackboard.getTargetScorerSpeed("target")));
    }

}
