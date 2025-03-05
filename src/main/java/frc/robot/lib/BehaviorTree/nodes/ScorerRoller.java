package frc.robot.lib.BehaviorTree.nodes;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerCommands;

public class ScorerRoller extends BehaviorTreeNode {
 double ScorerRollersSpeed; 
 Scorer m_Scorer;
 Command runScorerRollers;

    public ScorerRoller(Blackboard blackboard) {
    super(blackboard);

    }

    @Override
    public void initialize(){
       m_Scorer = RobotContainer.scorer;
        DoubleSupplier ScorerRollersSpeed = () -> blackboard.getTargetScorerSpeed("target");

        runScorerRollers = ScorerCommands.CS_setRunning(m_Scorer, ScorerRollersSpeed);
    }

    @Override
    public ExecutionStatus run() {
        if(!runScorerRollers.isScheduled() && runScorerRollers != null) {
            runScorerRollers.schedule();
        }

        if (runScorerRollers.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }

    @Override
    public void reset() {
        if (runScorerRollers.isScheduled()) {
            runScorerRollers.cancel();
            
        }
    }
    
    
}
