package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.subsystems.climber.ClimberCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeCommands;
import frc.robot.subsystems.scorer.Scorer;
import frc.robot.subsystems.scorer.ScorerCommands;
import frc.robot.subsystems.scorer.ScorerConstants;

public class ScoreCommands {

    //TODO: Make commands that utilize the elevator only able to run if the robot is in the safe zones on the field
    public static class ScoreL1 extends SequentialCommandGroup {
        public ScoreL1(Scorer m_scorer, Elevator m_elevator) {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    ElevatorCommands.EL_goToL1(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L1_HEIGHT - 2.0),
                    //ScorerCommands.CS_scoreCoral(m_scorer),
                    ScorerCommands.CS_shootL1SimCoral(m_scorer),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getCoral() == false)
            );
        }
    }

    public static class ScoreL2 extends SequentialCommandGroup {
        public ScoreL2(Scorer m_scorer, Elevator m_elevator) {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    ElevatorCommands.EL_goToL2(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L2_HEIGHT - 2.0),
                    //ScorerCommands.CS_scoreCoral(m_scorer),
                    ScorerCommands.CS_shootL2SimCoral(m_scorer),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getCoral() == false)
            );
        }
    }

    public static class ScoreL3 extends SequentialCommandGroup {
        public ScoreL3(Scorer m_scorer, Elevator m_elevator) {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    ElevatorCommands.EL_goToL3(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L3_HEIGHT - 2.0),
                    //ScorerCommands.CS_scoreCoral(m_scorer),
                    ScorerCommands.CS_shootL3SimCoral(m_scorer),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getCoral() == false)
                );
        }  
    }

    public static class ScoreL4 extends SequentialCommandGroup{
        public ScoreL4(Scorer m_scorer, Elevator m_elevator){
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    ElevatorCommands.EL_goToL4(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_L4_HEIGHT - 2.0),
                    ScorerCommands.CS_goToL4(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) >= ScorerConstants.CS_L4_ANGLE - 5.0),
                    ScorerCommands.CS_removeAlgae(m_scorer),
                    ScorerCommands.CS_shootL4SimCoral(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getCoral() == false)
            );
        }
    }

    public static class ScoreNet extends SequentialCommandGroup{
        //SIMULATION VERSION
        public ScoreNet(Scorer m_scorer, Elevator m_elevator, Intake m_intake, AbstractDriveTrainSimulation driveSimulation){
            addCommands(
                Commands.sequence(
                    IntakeCommands.IN_clearElevator(m_intake),
                    Commands.waitUntil(() -> m_intake.IN_getAngle().in(Degrees) <= -15.0),
                    ElevatorCommands.EL_goToNet(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    IntakeCommands.IN_rest(m_intake),
                    ScorerCommands.CS_net(m_scorer),
                    ScorerCommands.CS_shootSimAlgae(m_scorer,driveSimulation),
                    ScorerCommands.CS_removeAlgae(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getAlgae() == false) //Don't run if we don't have algae
            );
        }
        //NON-SIMULATION VERSION
        public ScoreNet(Scorer m_scorer, Elevator m_elevator, Intake m_intake){
            addCommands(
                Commands.sequence(
                    IntakeCommands.IN_clearElevator(m_intake),
                    Commands.waitUntil(() -> m_intake.IN_getAngle().in(Degrees) <= -15.0),
                    ElevatorCommands.EL_goToNet(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    IntakeCommands.IN_rest(m_intake),
                    ScorerCommands.CS_net(m_scorer),
                    ScorerCommands.CS_removeAlgae(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator))
                .unless(() -> m_scorer.CS_getAlgae() == false) //Don't run if we don't have algae
            );
        }
    }

    public static class IntakeGroundAlgae extends SequentialCommandGroup {
        public IntakeGroundAlgae(Scorer m_scorer, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeAlgae(m_intake), ScorerCommands.CS_intakeAlgae(m_scorer))
                    .until(() -> m_scorer.CS_getAlgae() == true),
                    Commands.parallel(IntakeCommands.IN_rest(m_intake), ScorerCommands.CS_stop(m_scorer))
                    )
            );
        }
    }

    public static class IntakeGroundAlgaeV2 extends SequentialCommandGroup {
        public IntakeGroundAlgaeV2(Scorer m_scorer, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeAlgae(m_intake), ScorerCommands.CS_intakeAlgae(m_scorer), IntakeCommands.IN_simSetRunning(m_intake)),
                    new WaitCommand(2),
                    Commands.parallel(IntakeCommands.IN_rest(m_intake), ScorerCommands.CS_stop(m_scorer), IntakeCommands.IN_simStopRunning(m_intake))
                    )
                );
        }
    }

    
    public static class IntakeCoral extends SequentialCommandGroup {
        public IntakeCoral(Scorer m_scorer, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeCoral(m_intake), ScorerCommands.CS_intakeCoral(m_scorer))
                    .until(() -> m_scorer.CS_getCoral() == true),
                    Commands.parallel(IntakeCommands.IN_rest(m_intake), ScorerCommands.CS_stop(m_scorer))
                    ).unless(() -> m_scorer.CS_getCoral() == true) //Don't run if we have a coral already
            );
        }
    }

    public static class RemoveAlgae extends SequentialCommandGroup {
        public RemoveAlgae(Scorer m_scorer, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    Commands.parallel(ScorerCommands.CS_removeAlgae(m_scorer),IntakeCommands.IN_outakeAlgae(m_intake))
                    .until(() -> m_scorer.CS_getAlgae() == false),
                    Commands.parallel(IntakeCommands.IN_rest(m_intake), ScorerCommands.CS_stop(m_scorer))
                    ).unless(() -> m_scorer.CS_getAlgae() == false)  //Don't run if we don't have algae
            ); 
        } 
    }

    public static class StartClimb extends SequentialCommandGroup {
        public StartClimb(Scorer m_scorer, Elevator m_elevator, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    ElevatorCommands.EL_goToRest(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_REST_HEIGHT + 2.0),
                    IntakeCommands.IN_clearElevator(m_intake),
                    ClimberCommands.CL_Extend(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == true))
                .unless(() -> m_scorer.CS_getCoral() == false)
            );
        }
    }

    public static class EndClimb extends SequentialCommandGroup {
        public EndClimb(Scorer m_scorer) {
            addCommands(
                Commands.sequence(
                    ClimberCommands.CL_Retract(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == false))
                .unless(() -> m_scorer.CS_getCoral() == false)
            );
        }
    }

    public static class StartClimbTest extends SequentialCommandGroup {
        public StartClimbTest() {
            addCommands(
                Commands.sequence(
                    ClimberCommands.CL_Extend(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == true))
            );
        }
    }

    public static class EndClimbTest extends SequentialCommandGroup {
        public EndClimbTest() {
            addCommands(
                Commands.sequence(
                    ClimberCommands.CL_Retract(RobotContainer.climber),
                    Commands.waitUntil(() -> RobotContainer.climber.CL_getExtended() == false))
            );
        }
    }

    public static class GrabL2Algae extends SequentialCommandGroup {
        public GrabL2Algae(Scorer m_scorer, Elevator m_elevator, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    ElevatorCommands.EL_goToL1(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    ScorerCommands.CS_goToAlgae(m_scorer),
                    ScorerCommands.CS_intakeAlgae(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAlgae() == true),
                    ScorerCommands.CS_stop(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator)
                    ).unless(() -> m_scorer.CS_getAlgae() == true) //Don't run if we have an algae already
            );
        }
    }

    public static class GrabL3Algae extends SequentialCommandGroup {
        public GrabL3Algae(Scorer m_scorer, Elevator m_elevator, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    ElevatorCommands.EL_goToL2(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    ScorerCommands.CS_goToAlgae(m_scorer),
                    ScorerCommands.CS_intakeAlgae(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAlgae() == true),
                    ScorerCommands.CS_stop(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator)
                    ).unless(() -> m_scorer.CS_getAlgae() == true) //Don't run if we have an algae already
            );
        }
    }

    public static class GrabAlgaeToRest extends SequentialCommandGroup {
        public GrabAlgaeToRest(Scorer m_scorer, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    Commands.parallel(IntakeCommands.IN_intakeAlgae(m_intake), ScorerCommands.CS_intakeAlgae(m_scorer))
                    .until(() -> m_scorer.CS_getAlgae() == true),
                    Commands.parallel(IntakeCommands.IN_rest(m_intake), ScorerCommands.CS_stop(m_scorer))
                    ).unless(() -> m_scorer.CS_getAlgae() == false) //Don't run if we don't have an algae already
            );
        }
    }

    public static class returnAllToRest extends SequentialCommandGroup {
        public returnAllToRest(Scorer m_scorer, Elevator m_elevator, Intake m_intake) {
            addCommands(
                Commands.sequence(
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 5.0),
                    ElevatorCommands.EL_goToRest(m_elevator)),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) <= ElevatorConstants.EL_REST_HEIGHT + 2.0),
                    IntakeCommands.IN_rest(m_intake)
            );
        }
    }

    // public static class NetTest extends SequentialCommandGroup {
    //     public NetTest(Scorer m_scorer, Elevator m_elevator) {
    //         addCommands(
    //             Commands.sequence(
    //                 ElevatorCommands.EL_goToMidpoint(m_elevator),
    //                 Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= 28),
    //                 ScorerCommands.CS_net(m_scorer),
    //                 Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) >= 160),
    //                 ElevatorCommands.EL_goToNet(m_elevator),
    //                 ScorerCommands.CS_removeAlgae(m_scorer),
    //                 ScorerCommands.CS_goToRest(m_scorer),
    //                 Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 10.0),
    //                 ElevatorCommands.EL_goToRest(m_elevator)
    //             )
    //         );
    //     }
    // }

    public static SequentialCommandGroup NetTest(Scorer m_scorer, Elevator m_elevator) {
        return new SequentialCommandGroup(
            ElevatorCommands.EL_goToMidpoint(m_elevator),
            Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= 16),
            ScorerCommands.CS_net(m_scorer),
            Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) >= 160),
            ElevatorCommands.EL_goToNet(m_elevator),
            Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= 46),
            new WaitCommand(0.3),
            ScorerCommands.CS_removeAlgae(m_scorer),
            ScorerCommands.CS_goToRest(m_scorer),
            Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 10.0),
            ElevatorCommands.EL_goToRest(m_elevator)
        );
    }

    public static SequentialCommandGroup L2AlgaeSequence(Scorer m_scorer, Elevator m_elevator) {
        return new SequentialCommandGroup(
            ElevatorCommands.EL_goToMidpoint(m_elevator),
            Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= 16),
            ScorerCommands.CS_runAlgaePosition(m_scorer),
            Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) >= 160),
            ElevatorCommands.EL_goToRest(m_elevator),
            ScorerCommands.CS_intakeAlgae(m_scorer)
        );
    }


    public static class NetTest extends SequentialCommandGroup {
        public NetTest(Scorer m_scorer, Elevator m_elevator) {
            addCommands(
                Commands.sequence(
                    ElevatorCommands.EL_goToNet(m_elevator),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= ElevatorConstants.EL_NET_HEIGHT - 2.0),
                    ScorerCommands.CS_net(m_scorer),
                    ScorerCommands.CS_removeAlgae(m_scorer),
                    ScorerCommands.CS_goToRest(m_scorer),
                    Commands.waitUntil(() -> m_scorer.CS_getAngle().in(Degrees) <= 10.0),
                    ElevatorCommands.EL_goToRest(m_elevator)
                )
            );
        }

    public static class ScoreTarget extends SequentialCommandGroup{
        public ScoreTarget(Scorer m_Scorer, Elevator m_elevator, Blackboard blackboard){
            addCommands(
                Commands.sequence(
                    ElevatorCommands.EL_setPositionToBlackboard(m_elevator, blackboard),
                    Commands.waitUntil(() -> m_elevator.getELPosition().in(Inches) >= blackboard.getTargetElevatorPosition("target") - 2.0),
                    ScorerCommands.CS_setPivotToBlackboard(m_Scorer, blackboard),
                    Commands.waitUntil(() -> m_Scorer.CS_getAngle().in(Degrees) >= blackboard.getTargetScorerAngle("target") - 4.0),
                    ScorerCommands.CS_setRunning(m_Scorer, () -> -0.3),
                    new WaitCommand(0.6),
                    ScorerCommands.CS_goToRest(m_Scorer),
                    ElevatorCommands.EL_goToRest(m_elevator),
                    ScorerCommands.CS_setRunning(m_Scorer, () -> 0.0)
                )
            );
        }
    }
   
    }
}
