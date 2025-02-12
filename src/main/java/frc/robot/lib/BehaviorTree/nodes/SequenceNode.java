package frc.robot.lib.BehaviorTree.nodes;

import java.util.ArrayList;
import java.util.List;

import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class SequenceNode extends BehaviorTreeNode {
    private final List<BehaviorTreeNode> children = new ArrayList<>();
    private static int currentIndex = 0;
    
        public SequenceNode(Blackboard blackboard) {
            super(blackboard);
        }

        @Override
        public void initialize() {
            for (BehaviorTreeNode child : children) {
                child.initialize();
            }
        }
    
        public void addChild(BehaviorTreeNode child) {
            child.setParent(this);
            children.add(child);
        }
    
        @Override
        public ExecutionStatus run() {

            while (currentIndex < (children.size())) {
                
                ExecutionStatus status = children.get(currentIndex).execute();
    
                if (status == ExecutionStatus.RUNNING || status == ExecutionStatus.FAILURE) {
                    return status;
                }
    
                currentIndex++;
            }
    
            currentIndex = 0; // Reset for next execution
            return ExecutionStatus.SUCCESS;
        }
        
    @Override
    public void reset() {
        currentIndex = 0;
        for (int i = 0; i < children.size(); i++) {
            children.get(i).reset();
        }
    }
}