package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerCommands;

public class CombinedSubsystemCommands {

    public static ParallelCommandGroup scoreAtL1(Elevator m_Elevator, Scorer m_Scorer){
        return new ParallelCommandGroup(ElevatorCommands.EL_goToL1(m_Elevator), ScorerCommands.CS_goToL1(m_Scorer));
    }

    public static ParallelCommandGroup goToRest(Elevator m_Elevator, Scorer m_Scorer){
        return new ParallelCommandGroup(ElevatorCommands.EL_goToRest(m_Elevator), ScorerCommands.CS_goToRest(m_Scorer));
    }
    
}
