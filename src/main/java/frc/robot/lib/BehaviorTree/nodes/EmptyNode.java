package frc.robot.lib.BehaviorTree.nodes;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class EmptyNode extends BehaviorTreeNode {

    public EmptyNode(Blackboard blackboard) {
    super(blackboard);
    }

    @Override
    public void initialize(){

    }

    @Override
    public ExecutionStatus run() {
        return ExecutionStatus.SUCCESS;
    }
    
    
}
