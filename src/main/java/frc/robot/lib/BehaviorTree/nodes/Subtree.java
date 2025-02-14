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

    @Override
    public void initialize() {
        for (BehaviorTreeNode child: children) {
            child.initialize();
        }
    }

    public void addChild(BehaviorTreeNode child) {
        children.add(child);
    }

    @Override
    public ExecutionStatus run() {
        for (BehaviorTreeNode child : children) {
            child.initialize();
            ExecutionStatus status = child.run();
            if (status != ExecutionStatus.SUCCESS) {
                return status;
            } 
        }
        return ExecutionStatus.SUCCESS;
    }


    
}
