package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.util.AllianceFlipUtil;

public class ScorerRoller extends BehaviorTreeNode {
 double ScorerRollersSpeed; 

    public ScorerRoller(Blackboard blackboard) {
    super(blackboard);

    }

    @Override
    public void initialize(){
        ScorerRollersSpeed = blackboard.getTargetIntakeSpeed("target");

    }

    @Override
    public ExecutionStatus run() {
        return ExecutionStatus.SUCCESS;
    }
    
    
}
