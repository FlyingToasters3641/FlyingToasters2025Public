package frc.robot.lib.BehaviorTree.nodes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class SequenceNode extends BehaviorTreeNode {
    private final List<BehaviorTreeNode> children = new ArrayList<>();
    private int currentIndex = 0;

    public SequenceNode(Blackboard blackboard) {
        super(blackboard);
    }

    public void addChild(BehaviorTreeNode child) {
        children.add(child);
    }

    @Override
    public ExecutionStatus execute() {
        while (currentIndex < children.size()) {
            ExecutionStatus status = children.get(currentIndex).execute();

            if (status == ExecutionStatus.RUNNING || status == ExecutionStatus.FAILURE) {
                return status;
            }

            currentIndex++;
        }

        currentIndex = 0; // Reset for next execution
        return ExecutionStatus.SUCCESS;
    }
}