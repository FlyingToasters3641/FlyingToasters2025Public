package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.util.AllianceFlipUtil;

public class DriveToIntake extends BehaviorTreeNode {
    Command driveToCommand;
    private Pose2d pose;
    boolean isAlgae;

    public DriveToIntake(Blackboard blackboard) {
        super(blackboard);

    }

    @Override
    public void initialize() {
        isAlgae = blackboard.isTargetAlgae("target");
        if (isAlgae) {
            //TODO: change both constants for intaking to actual stuff 
            pose = Constants.reefBranchJ;
        } else {
            if (blackboard.isPlayerStationLeft()) {
                pose = Constants.humanPlayerStationLeft;
            } else {
            pose = Constants.humanPlayerStationRight;
        }
    }
        pose = AllianceFlipUtil.apply(pose);
        driveToCommand = AutoBuilder.pathfindToPose(pose, Constants.constraints);
        
    }

    @Override
    public ExecutionStatus run() {
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
