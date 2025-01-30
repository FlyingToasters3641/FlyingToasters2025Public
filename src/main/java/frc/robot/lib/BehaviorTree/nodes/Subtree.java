package frc.robot.lib.BehaviorTree.nodes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class Subtree extends BehaviorTreeNode {
    private final List<BehaviorTreeNode> children = new ArrayList<>();


    public Subtree(Blackboard blackboard) {
    super(blackboard);
    }

    public void addChild(BehaviorTreeNode child) {
        children.add(child);
    }

    @Override
    public ExecutionStatus execute() {
        for (BehaviorTreeNode child : children) {
            ExecutionStatus status = child.execute();
            if (status != ExecutionStatus.SUCCESS) {
                return status;
            } 
        }
        return ExecutionStatus.SUCCESS;
    }


    
}
