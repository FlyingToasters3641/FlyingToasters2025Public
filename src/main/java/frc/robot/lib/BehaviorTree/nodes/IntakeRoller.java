package frc.robot.lib.BehaviorTree.nodes;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.util.AllianceFlipUtil;

public class IntakeRoller extends BehaviorTreeNode {
 double IntakeRollersSpeed;
 Intake m_Intake;
 Command runIntakeRollers;


    public IntakeRoller(Blackboard blackboard) {
    super(blackboard);

    }

    @Override
    public void initialize(){
        IntakeRollersSpeed = blackboard.getTargetIntakeSpeed("target");

        m_Intake = RobotContainer.intake;
        runIntakeRollers = IntakeCommands.IN_setRunning(m_Intake, IntakeRollersSpeed);

    }

    @Override
    public ExecutionStatus run() {
         if(!runIntakeRollers.isScheduled() && runIntakeRollers != null) {
            runIntakeRollers.schedule();
        }

        if (runIntakeRollers.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }

    @Override
    public void reset() {
        if (runIntakeRollers.isScheduled()) {
            runIntakeRollers.cancel();
            
        }
    }
    
    
}
