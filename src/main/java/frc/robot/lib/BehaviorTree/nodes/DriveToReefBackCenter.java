package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;

public class DriveToReefBackCenter extends BehaviorTreeNode {
    Command driveToHumanPlayerStationRight = AutoBuilder.pathfindToPose(Constants.targetPose, Constants.constraints);

    public DriveToReefBackCenter(Blackboard blackboard) {
        super(blackboard);
    }

    @Override
    public ExecutionStatus execute() {
        if(!driveToHumanPlayerStationRight.isScheduled()) {
            driveToHumanPlayerStationRight.schedule();
        }

        if (driveToHumanPlayerStationRight.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }
    
    @Override
    public void reset() {
        if (driveToHumanPlayerStationRight.isScheduled()) {
            driveToHumanPlayerStationRight.cancel();
            
        }
    }
}
