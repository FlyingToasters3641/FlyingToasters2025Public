package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.scorer.ScorerCommands;

public class ScoreL4 extends SequentialCommandGroup{
    public ScoreL4(){
        addCommands(
            Commands.sequence(
                ScorerCommands.CS_goToRest(RobotContainer.scorer),
                ElevatorCommands.EL_goToL4(RobotContainer.elevator),
                Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                ScorerCommands.CS_goToL4(RobotContainer.scorer),
                ScorerCommands.CS_removeAlgae(RobotContainer.scorer),
                ScorerCommands.CS_goToRest(RobotContainer.scorer),
                Commands.waitUntil(() -> RobotContainer.scorer.CS_getAngle().in(Degrees) <= 5.0),
                ElevatorCommands.EL_goToRest(RobotContainer.elevator))
            .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
        );
    }
}
