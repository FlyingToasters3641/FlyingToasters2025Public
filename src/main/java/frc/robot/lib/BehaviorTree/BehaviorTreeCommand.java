package frc.robot.lib.BehaviorTree;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;

public class BehaviorTreeCommand extends Command {
    private final BehaviorTreeNode root;
    private boolean isCompleted = false;

    public BehaviorTreeCommand(BehaviorTreeNode root) {
        this.root = root;
    }

    @Override
    public void initialize() {
        isCompleted = false;
    }

    @Override
    public void execute() {
        if(!isCompleted) {
            ExecutionStatus status = root.execute();
            if(status == ExecutionStatus.SUCCESS || status == ExecutionStatus.FAILURE) {
                isCompleted = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return isCompleted;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("BehaviorTreeCommand was interrupted.");
        } else {
            System.out.println("BehaviorTreeCommand completed successfully.");
        }
    }
}
