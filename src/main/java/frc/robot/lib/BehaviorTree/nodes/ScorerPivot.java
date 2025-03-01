package frc.robot.lib.BehaviorTree.nodes;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.subsystems.scorer.Scorer;

public class ScorerPivot extends BehaviorTreeNode {
    double ScorerPivotPosition; 
    Scorer m_Scorer;
    Command runScorerAngle;
   
       public ScorerPivot(Blackboard blackboard) {
       super(blackboard);
   
       }
   
       @Override
       public void initialize(){
        m_Scorer = RobotContainer.scorer;
        ScorerPivotPosition = blackboard.getTargetScorerAngle("target");
        runScorerAngle = m_Scorer.CS_runSetpoint(Degrees.of(ScorerPivotPosition));
           
       }
   
       @Override
       public ExecutionStatus run() {
        if(!runScorerAngle.isScheduled() && runScorerAngle != null) {
            runScorerAngle.schedule();
        }

        if (runScorerAngle.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }

    @Override
    public void reset() {
        if (runScorerAngle.isScheduled()) {
            runScorerAngle.cancel();
            
        }
    }
}
   
