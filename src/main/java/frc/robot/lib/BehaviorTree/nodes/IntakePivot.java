package frc.robot.lib.BehaviorTree.nodes;

import static edu.wpi.first.units.Units.Degrees;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.subsystems.intake.Intake;
import frc.robot.util.AllianceFlipUtil;

public class IntakePivot extends BehaviorTreeNode {
 double IntakePivotPosition; 
 Intake m_Intake;
 Command runIntakeAngle;

    public IntakePivot(Blackboard blackboard) {
    super(blackboard);

    }

    @Override
    public void initialize(){
        m_Intake = RobotContainer.intake;
        IntakePivotPosition = blackboard.getTargetIntakeAngle("target");
        runIntakeAngle= m_Intake.IN_runSetpoint(Degrees.of(IntakePivotPosition));
    }
    public ExecutionStatus run() {
        if(!runIntakeAngle.isScheduled() && runIntakeAngle != null) {
            runIntakeAngle.schedule();
        }

        if (runIntakeAngle.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }
    
    @Override
    public void reset() {
        if (runIntakeAngle.isScheduled()) {
            runIntakeAngle.cancel();
            
        }
    }
}
