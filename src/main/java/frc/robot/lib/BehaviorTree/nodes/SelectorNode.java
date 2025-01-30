package frc.robot.lib.BehaviorTree.nodes;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class SelectorNode extends BehaviorTreeNode{
    private final List<ChildNodeConditionPair> children = new ArrayList<>();

    public SelectorNode(Blackboard blackboard){
        super(blackboard);
    }

    public void addChild(BehaviorTreeNode child, Predicate<Blackboard> condition){
        children.add(new ChildNodeConditionPair(child, condition));
    }

    @Override
    public ExecutionStatus run() {
        for (ChildNodeConditionPair pair : children) {
            if (pair.condition.test(blackboard)) {
                ExecutionStatus status = pair.child.run();
                if (status == ExecutionStatus.RUNNING || status == ExecutionStatus.SUCCESS) {
                    return status;
                }
            }
        }
        return ExecutionStatus.FAILURE;
    }
    
    private static class ChildNodeConditionPair {
        BehaviorTreeNode child;
        Predicate<Blackboard> condition;

        ChildNodeConditionPair(BehaviorTreeNode child, Predicate<Blackboard> condition){
            this.child = child;
            this.condition = condition;
        }   
    }

    @Override
    public void reset() {
        for (ChildNodeConditionPair pair : children) {
            if (pair.child instanceof SequenceNode) {
                ((SequenceNode) pair.child).reset();
            }
            // Add reset logic for other node types if necessary
        }
    }

}
