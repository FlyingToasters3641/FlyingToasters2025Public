package frc.robot.lib.BehaviorTree.nodes;

import java.util.function.Predicate;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class InfiniteLoopNode extends BehaviorTreeNode {
    private final BehaviorTreeNode child;
    private final Predicate<Blackboard> condition;

    public InfiniteLoopNode(Blackboard blackboard, BehaviorTreeNode child, Predicate<Blackboard> condition) {
        super(blackboard);
        this.child = child;
        this.condition = condition;
    }

    @Override
    public ExecutionStatus execute() {
        if(condition.test(blackboard)) {
            return ExecutionStatus.SUCCESS;
        }

        ExecutionStatus status = child.execute();

        if (status == ExecutionStatus.FAILURE){
            return ExecutionStatus.FAILURE;
        } 

        return ExecutionStatus.RUNNING;
    }

    @Override
    public void reset() {
        child.reset();
    }   
}
