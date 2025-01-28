package frc.robot.lib.BehaviorTree.nodes;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public abstract class BehaviorTreeNode {
    protected Blackboard blackboard;

    public BehaviorTreeNode(Blackboard blackboard) {
        this.blackboard = blackboard;
    }

    public abstract ExecutionStatus execute();

    public void reset() {
        // Do nothing by default
    }
}