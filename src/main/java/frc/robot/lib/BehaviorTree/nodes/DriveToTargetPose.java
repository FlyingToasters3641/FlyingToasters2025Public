package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class DriveToTargetPose extends BehaviorTreeNode {
    Command driveToCommand;
    private Pose2d pose;

    public DriveToTargetPose(Blackboard blackboard) {
        super(blackboard);
    }

    @Override
    public void initialize() {
        if ((blackboard.getTargetPose("target")) != null) {
            pose = blackboard.getTargetPose("target");
            driveToCommand = AutoBuilder.pathfindToPose(pose, Constants.constraints);
            } else {
                driveToCommand = null;
            }
    }

    @Override
    public ExecutionStatus run() {
        
        if(!driveToCommand.isScheduled() && driveToCommand != null) {
            driveToCommand.schedule();
        }
    
        if (driveToCommand.isFinished()) {
            blackboard.set("isDone", true);
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
