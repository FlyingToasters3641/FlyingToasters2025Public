package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class DriveToPose extends BehaviorTreeNode {
    Command driveToCommand;
    private Pose2d pose;

    public DriveToPose(Blackboard blackboard, Pose2d pose) {
        super(blackboard);
        this.pose = pose;
    }

    @Override
    public ExecutionStatus run() {
        driveToCommand = AutoBuilder.pathfindToPose(pose, Constants.constraints);
        if(!driveToCommand.isScheduled() && driveToCommand != null) {
            driveToCommand.schedule();
        }

        if (driveToCommand.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }
    
    @Override
    public void reset() {
        if (driveToCommand.isScheduled()) {
            driveToCommand.cancel();
            
        }
    }
}
