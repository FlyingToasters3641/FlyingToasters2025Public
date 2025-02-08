package frc.robot.lib.BehaviorTree.nodes;

import frc.robot.lib.BehaviorTree.BehaviorTreeDebugger;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public abstract class BehaviorTreeNode {
    protected Blackboard blackboard;
    protected BehaviorTreeDebugger debugger = BehaviorTreeDebugger.getInstance();
    protected BehaviorTreeNode parent = null; // Track parent node

    public BehaviorTreeNode(Blackboard blackboard) {
        this.blackboard = blackboard;
    }

    public ExecutionStatus execute() {
        ExecutionStatus status = run();
        debugger.logNodeExecution(this, status, parent);
        return status;
    }

    public void start() {
        initialize();
    }

    public abstract void initialize();

    public void reset() {
        // Do nothing by default
    }

    public void setParent(BehaviorTreeNode parent) {
        this.parent = parent;
    }

    protected abstract ExecutionStatus run();
}