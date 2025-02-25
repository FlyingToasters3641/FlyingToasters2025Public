package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.util.AllianceFlipUtil;

public class IntakeRoller extends BehaviorTreeNode {
 double IntakeRollersSpeed; 

    public IntakeRoller(Blackboard blackboard) {
    super(blackboard);

    }

    @Override
    public void initialize(){
        IntakeRollersSpeed = blackboard.getTargetIntakeSpeed("target");

    }

    @Override
    public ExecutionStatus run() {
        return ExecutionStatus.SUCCESS;
    }
    
    
}
