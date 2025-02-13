package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.scorer.ScorerCommands;

public class ScoreL1 extends SequentialCommandGroup {
    public ScoreL1() {
        addCommands(
            Commands.sequence(
                ScorerCommands.CS_goToRest(RobotContainer.scorer),
                ElevatorCommands.EL_goToL1(RobotContainer.elevator),
                Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L1_HEIGHT - 2.0),
                ScorerCommands.CS_scoreCoral(RobotContainer.scorer),
                ElevatorCommands.EL_goToRest(RobotContainer.elevator))
            .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
        );
    }
    
}
