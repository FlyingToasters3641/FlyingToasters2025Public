package frc.robot.lib.BehaviorTree.nodes;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.ExecutionStatus;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorPosition extends BehaviorTreeNode {
    double ElevatorPosition; 
    Elevator m_Elevator;
   Command runElevator;
       public ElevatorPosition(Blackboard blackboard) {
       super(blackboard);
       }
   
       @Override
       public void initialize(){
        ElevatorPosition = blackboard.getTargetElevatorPosition("target");
        m_Elevator = RobotContainer.elevator;
        runElevator = Commands.runOnce(() -> m_Elevator.EL_setpoint = Inches.of(ElevatorPosition));
       
           
       }
   
        @Override
    public ExecutionStatus run() {
        if(!runElevator.isScheduled() && runElevator != null) {
            runElevator.schedule();
        }

        if (runElevator.isFinished()) {
            return ExecutionStatus.SUCCESS;
        }

        return ExecutionStatus.RUNNING;
    }
    
    @Override
    public void reset() {
        if (runElevator.isScheduled()) {
            runElevator.cancel();
            
        }
    }

       
       
   }