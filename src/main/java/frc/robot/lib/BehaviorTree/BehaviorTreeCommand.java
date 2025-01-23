package frc.robot.lib.BehaviorTree;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;

public class BehaviorTreeCommand extends Command {
    private final BehaviorTreeNode root;

    public BehaviorTreeCommand(BehaviorTreeNode root) {
        this.root = root;
    }

    @Override
    public void initialize() {
        // Optionally reset the tree
    }

    @Override
    public void execute() {
        root.execute();
    }

    @Override
    public boolean isFinished() {
        return root.execute() == ExecutionStatus.SUCCESS;
    }

    @Override
    public void end(boolean interrupted) {
        // Cleanup if necessary
    }
}
