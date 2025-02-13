package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.scorer.ScorerCommands;

public class ScoreCommands {
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

    public class ScoreL2 extends SequentialCommandGroup {
        public ScoreL2() {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(RobotContainer.scorer),
                    ElevatorCommands.EL_goToL2(RobotContainer.elevator),
                    Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L2_HEIGHT - 2.0),
                    ScorerCommands.CS_scoreCoral(RobotContainer.scorer),
                    ElevatorCommands.EL_goToRest(RobotContainer.elevator))
                .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
            );
        }
    }

    public class ScoreL3 extends SequentialCommandGroup {
        public ScoreL3() {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(RobotContainer.scorer),
                    ElevatorCommands.EL_goToL3(RobotContainer.elevator),
                    Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L2_HEIGHT - 2.0),
                    ScorerCommands.CS_scoreCoral(RobotContainer.scorer),
                    ElevatorCommands.EL_goToRest(RobotContainer.elevator))
                .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
            );
        }  
    }

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

    public static class ScoreNet extends SequentialCommandGroup{
        public ScoreNet(){
            addCommands(
                Commands.sequence(
                    IntakeCommands.IN_clearElevator(RobotContainer.intake),
                    Commands.waitUntil(() -> RobotContainer.intake.IN_getAngle().in(Degrees) <= -15.0),
                    ElevatorCommands.EL_goToNet(RobotContainer.elevator),
                    Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    IntakeCommands.IN_rest(RobotContainer.intake),
                    ScorerCommands.CS_net(RobotContainer.scorer),
                    ScorerCommands.CS_removeAlgae(RobotContainer.scorer),
                    ScorerCommands.CS_goToRest(RobotContainer.scorer),
                    Commands.waitUntil(() -> RobotContainer.scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(RobotContainer.elevator))
                .unless(() -> RobotContainer.scorer.CS_getAlgae() == false) //Don't run if we don't have algae
            );
        }
    }

    public class intakeGroundAlgae extends SequentialCommandGroup {
        public intakeGroundAlgae() {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeAlgae(RobotContainer.intake), ScorerCommands.CS_intakeAlgae(RobotContainer.scorer))
                    .until(() -> RobotContainer.scorer.CS_getAlgae() == true),
                    Commands.parallel(IntakeCommands.IN_rest(RobotContainer.intake), ScorerCommands.CS_stop(RobotContainer.scorer))
                    ).unless(() -> RobotContainer.scorer.CS_getAlgae() == true) //Don't run if we have an algae already
            );
        }
    }

    public class intakeCoral extends SequentialCommandGroup {
        public intakeCoral() {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeCoral(RobotContainer.intake), ScorerCommands.CS_intakeCoral(RobotContainer.scorer))
                    .until(() -> RobotContainer.scorer.CS_getCoral() == true),
                    Commands.parallel(IntakeCommands.IN_rest(RobotContainer.intake), ScorerCommands.CS_stop(RobotContainer.scorer))
                    ).unless(() -> RobotContainer.scorer.CS_getCoral() == true) //Don't run if we have a coral already
            );
        }
    }

    public class removeAlgae extends SequentialCommandGroup {
        public removeAlgae() {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_removeAlgae(RobotContainer.scorer).alongWith(IntakeCommands.IN_outakeAlgae(RobotContainer.intake)
                    .until(() -> RobotContainer.scorer.CS_getAlgae() == false),
                    Commands.parallel(IntakeCommands.IN_rest(RobotContainer.intake), ScorerCommands.CS_stop(RobotContainer.scorer)))
                    ).unless(() -> RobotContainer.scorer.CS_getAlgae() == false)  //Don't run if we don't have algae
            ); 
        } 
    }

    public class startClimb extends SequentialCommandGroup {
        public startClimb() {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(RobotContainer.scorer),
                    ElevatorCommands.EL_goToRest(RobotContainer.elevator),
                    Commands.waitUntil(() -> RobotContainer.elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_REST_HEIGHT + 2.0),
                    IntakeCommands.IN_clearElevator(RobotContainer.intake),
                    ClimberCommands.CL_Extend(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == true))
                .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
            );
        }
    }

    public class endClimb extends SequentialCommandGroup {
        public endClimb() {
            addCommands(
                Commands.sequence(
                    ClimberCommands.CL_Retract(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == false))
                .unless(() -> RobotContainer.scorer.CS_getCoral() == false)
            );
        }
    }
}
