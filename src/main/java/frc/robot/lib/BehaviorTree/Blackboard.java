package frc.robot.lib.BehaviorTree;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.trees.Targets;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.scorer.ScorerConstants;

public class Blackboard {
    private final ConcurrentHashMap<String, Object> data = new ConcurrentHashMap<>();

    public void set(String key, Object value) {
        data.put(key, value);
    }

    public Object get(String key) {
        return data.get(key);
    }

    public Targets getTarget(String key) {
        if ((Targets)data.get(key) != null) {
        return (Targets) data.get(key);
        } else {
        return Targets.NONE;
        }

    }

    public boolean containsKey(String key) {
        return data.containsKey(key);
    }

    public boolean isPlayerStationLeft() {
        if (data.get("playerStation") == "left") {
            return true;
        } else {
            return false;
        }
    }

    public boolean getBoolean(String key) {
        if (data.get(key) == null) {
            return false;
        } else {
        return (boolean) data.get(key);
        }
    }

    public boolean isTargetAlgae(String key) {
        Targets currentTarget;
        if ((Targets)data.get(key) != null) {
        currentTarget = (Targets)data.get(key);
        } else {
        currentTarget = Targets.NONE;
        }
        boolean Algae;
        if (currentTarget != null) {
        switch (currentTarget) {
            case PROCESSOR:
                Algae = true;
                break;
            case NET:
                Algae = true;
                break;
            default:
                Algae = false;
                break;
        }
    } else {
        Algae = false;
    }
        return Algae;

    }

    public Pose2d getTargetPose(String key) {
        Targets currentTarget;
        if ((Targets)data.get(key) != null) {
        currentTarget = (Targets)data.get(key);
        } else {
        currentTarget = Targets.NONE;
        }
        Pose2d targetPose = new Pose2d();
        if (currentTarget != null) {
            switch (currentTarget) {
                case PROCESSOR:
                    targetPose = Constants.processorPose;
                    break;
                case NET:
                    targetPose = Constants.netPose;
                    break;
                case A1:
                    targetPose = Constants.reefBranchA;
                    break;
                case A2:
                    targetPose = Constants.reefBranchA;
                    break;
                case A3:
                    targetPose = Constants.reefBranchA;
                    break;
                case A4:
                    targetPose = Constants.reefBranchA;
                    break;
                case B1:
                    targetPose = Constants.reefBranchB;
                    break;
                case B2:
                    targetPose = Constants.reefBranchB;
                    break;
                case B3:
                    targetPose = Constants.reefBranchB;
                    break;
                case B4:
                    targetPose = Constants.reefBranchB;
                    break;
                case C1:
                    targetPose = Constants.reefBranchC;
                    break;
                case C2:
                    targetPose = Constants.reefBranchC;
                    break;
                case C3:
                    targetPose = Constants.reefBranchC;
                    break;
                case C4:
                    targetPose = Constants.reefBranchC;
                    break;
                case D1:
                    targetPose = Constants.reefBranchD;
                    break;
                case D2:
                    targetPose = Constants.reefBranchD;
                    break;
                case D3:
                    targetPose = Constants.reefBranchD;
                    break;
                case D4:
                    targetPose = Constants.reefBranchD;
                    break;
                case E1:
                    targetPose = Constants.reefBranchE;
                    break;
                case E2:
                    targetPose = Constants.reefBranchE;
                    break;
                case E3:
                    targetPose = Constants.reefBranchE;
                    break;
                case E4:
                    targetPose = Constants.reefBranchE;
                    break;
                case F1:
                    targetPose = Constants.reefBranchF;
                    break;
                case F2:
                    targetPose = Constants.reefBranchF;
                    break;
                case F3:
                    targetPose = Constants.reefBranchF;
                    break;
                case F4:
                    targetPose = Constants.reefBranchF;
                    break;
                case G1:
                    targetPose = Constants.reefBranchG;
                    break;
                case G2:
                    targetPose = Constants.reefBranchG;
                    break;
                case G3:
                    targetPose = Constants.reefBranchG;
                    break;
                case G4:
                    targetPose = Constants.reefBranchG;
                    break;
                case H1:
                    targetPose = Constants.reefBranchH;
                    break;
                case H2:
                    targetPose = Constants.reefBranchH;
                    break;
                case H3:
                    targetPose = Constants.reefBranchH;
                    break;
                case H4:
                    targetPose = Constants.reefBranchH;
                    break;
                case I1:
                    targetPose = Constants.reefBranchI;
                    break;
                case I2:
                    targetPose = Constants.reefBranchI;
                    break;
                case I3:
                    targetPose = Constants.reefBranchI;
                    break;
                case I4:
                    targetPose = Constants.reefBranchI;
                    break;
                case J1:
                    targetPose = Constants.reefBranchJ;
                    break;
                case J2:
                    targetPose = Constants.reefBranchJ;
                    break;
                case J3:
                    targetPose = Constants.reefBranchJ;
                    break;
                case J4:
                    targetPose = Constants.reefBranchJ;
                    break;
                case K1:
                    targetPose = Constants.reefBranchK;
                    break;
                case K2:
                    targetPose = Constants.reefBranchK;
                    break;
                case K3:
                    targetPose = Constants.reefBranchK;
                    break;
                case K4:
                    targetPose = Constants.reefBranchK;
                    break;
                case L1:
                    targetPose = Constants.reefBranchL;
                    break;
                case L2:
                    targetPose = Constants.reefBranchL;
                    break;
                case L3:
                    targetPose = Constants.reefBranchL;
                    break;
                case L4:
                    targetPose = Constants.reefBranchL;
                    break;
                case TEST:
                    targetPose = Constants.testPose1;
                    break;
                default:
                    targetPose = null;
                    break;
            }
        } else {
            targetPose = null;
        }
        return targetPose;
    }
    //these are the poses for the intake pivot
    public double getTargetIntakeAngle(String key) {
        Targets currentTarget = (Targets) data.get(key);
        double targetAngle = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
                case PROCESSOR:
                targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case NET:
            targetAngle = IntakeConstants.INTAKE_CLEAR_ELEVATOR_ANGLE;
                break;
            case A1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case A2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case A3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case A4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case B1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case B2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case B3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case B4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case C1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case C2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case C3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case C4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case D1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case D2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case D3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case D4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case E1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case E2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case E3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case E4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case F1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case F2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case F3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case F4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case G1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case G2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case G3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case G4:
                targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case H1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case H2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case H3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case H4:
                targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case I1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case I2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case I3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case I4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case J1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case J2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case J3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case J4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case K1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case K2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case K3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case K4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case L1:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case L2:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case L3:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case L4:
            targetAngle = IntakeConstants.INTAKE_REST_ANGLE;
                break;
            case INTAKE_ALGAE:
            targetAngle = IntakeConstants.INTAKE_ALGAE_ANGLE;
                break;
            case INTAKE_CORAL:
            targetAngle = IntakeConstants.INTAKE_CORAL_ANGLE;
            default:
            targetAngle = 0;
                break;
            }
        } else {
            targetAngle = 0;
        }

        return targetAngle;
    }

    public double getTargetIntakeSpeed(String key) {
        Targets currentTarget = (Targets) data.get(key);
        double targetSpeed = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
            case PROCESSOR:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case NET:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
                case INTAKE_ALGAE:
                    targetSpeed = IntakeConstants.INTAKE_ALGAE_SPEED;
                case INTAKE_CORAL:
                    targetSpeed = IntakeConstants.INTAKE_CORAL_SPEED;
                    break;
                case A1:
                    targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                    break;
                case A2:
                    targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                case A3: 
                    targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                case A4:
                    targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
                 case B1:
                    targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case B2:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case B3:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case B4:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case C1:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case C2:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case C3:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case C4:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case D1:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case D2:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case D3:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case D4:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case E1:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case E2:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case E3:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case E4:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case F1:
            targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case F2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case F3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case F4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case G1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case G2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case G3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case G4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case H1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case H2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case H3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case H4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case I1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case I2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case I3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case I4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case J1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case J2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case J3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case J4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case K1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case K2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case K3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case K4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case L1:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case L2:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case L3:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            case L4:
                targetSpeed = IntakeConstants.INTAKE_REST_SPEED;
                break;
            default:
                targetSpeed = 0;
                break;
            }
        } else {
            targetSpeed = 0;
        }

        return targetSpeed;
    } 

    
     public double getTargetScorerSpeed(String key) {
        Targets currentTarget = (Targets) data.get(key);
        double targetSpeed = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
            case PROCESSOR:
                targetSpeed = ScorerConstants.SCORER_REMOVE_ALGAE;
                break;
            case NET:
            targetSpeed = ScorerConstants.SCORER_STOP;
                break;
                case INTAKE_ALGAE:
                    targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                case INTAKE_CORAL:
                    targetSpeed = ScorerConstants.SCORER_INTAKE_CORAL;
                    break;
                case A1:
                    targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                    break;
                case A2:
                    targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                    break;
                case A3: 
                    targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                    break;
                case A4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
                 case B1:
                 targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case B2:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case B3:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case B4:
            targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case C1:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case C2:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case C3:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case C4:
            targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case D1:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case D2:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case D3:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case D4:
            targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case E1:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case E2:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case E3:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case E4:
            targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case F1:
            targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case F2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case F3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case F4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case G1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case G2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case G3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case G4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case H1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case H2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case H3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case H4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case I1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case I2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case I3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case I4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case J1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case J2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case J3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case J4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case K1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case K2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case K3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case K4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            case L1:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case L2:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case L3:
                targetSpeed = ScorerConstants.SCORER_CORAL_SPEED;
                break;
            case L4:
                targetSpeed = ScorerConstants.SCORER_CORAL_L4_SPEED;
                break;
            default:
                targetSpeed = 0;
                break;
            }
        } else {
            targetSpeed = 0;
        }

        return targetSpeed;
    }  

    public double getTargetScorerAngle(String key) {
        Targets currentTarget = (Targets) data.get(key);
        double targetAngle = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
                case PROCESSOR:
                targetAngle = ScorerConstants.SCORER_REST_ANGLE;
                break;
            case NET:
            targetAngle = ScorerConstants.SCORER_REST_NET;
                break;
            case A1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case A2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case A3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case A4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case B1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case B2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case B3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case B4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case C1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case C2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case C3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case C4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case D1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case D2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case D3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case D4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case E1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case E2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case E3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case E4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case F1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case F2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case F3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case F4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case G1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case G2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case G3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case G4:
                targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case H1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case H2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case H3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case H4:
                targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case I1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case I2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case I3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case I4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case J1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case J2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case J3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case J4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case K1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case K2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case K3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case K4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case L1:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case L2:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case L3:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case L4:
            targetAngle = ScorerConstants.SCORER_GO_TO_L4_ANGLE;
                break;
            case CS_SCORE_CORAL:
            targetAngle = ScorerConstants.SCORER_GO_TO_CORAL_ANGLE;
                break;
            case CS_REMOVE_ALGAE:
            targetAngle = ScorerConstants.SCORER_GO_TO_ALGAE_ANGLE;
            default:
            targetAngle = 0;
                break;
            }
        } else {
            targetAngle = 0;
        }

        return targetAngle;
    }

 public double getTargetElevatorPosition(String key) {
        Targets currentTarget = (Targets) data.get(key);
        double targetPosition = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
            case PROCESSOR:
                targetPosition = ElevatorConstants.EL_REST_HEIGHT;
                break;
            case NET:
            targetPosition = ElevatorConstants.EL_NET_HEIGHT;
                break;
                case INTAKE_ALGAE:
                    targetPosition = ElevatorConstants.EL_REST_HEIGHT;
                case INTAKE_CORAL:
                targetPosition = ElevatorConstants.EL_REST_HEIGHT;
                    break;
                case A1:
                targetPosition = ElevatorConstants.EL_L1_HEIGHT; 
                    break;
                case A2:
                targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
                case A3: 
                targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
                case A4:
                targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
                 case B1:
                 targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case B2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case B3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case B4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case C1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case C2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case C3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case C4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case D1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case D2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case D3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case D4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case E1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case E2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case E3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case E4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case F1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case F2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case F3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case F4:
                targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case G1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case G2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case G3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case G4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case H1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case H2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case H3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case H4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case I1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case I2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case I3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case I4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case J1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case J2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case J3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case J4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case K1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case K2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case K3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case K4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            case L1:
            targetPosition = ElevatorConstants.EL_L1_HEIGHT;
                break;
            case L2:
            targetPosition = ElevatorConstants.EL_L2_HEIGHT;
                break;
            case L3:
            targetPosition = ElevatorConstants.EL_L3_HEIGHT;
                break;
            case L4:
            targetPosition = ElevatorConstants.EL_L4_HEIGHT;
                break;
            default:
            targetPosition = ElevatorConstants.EL_REST_HEIGHT;
                break;
            }
        } else {
            targetPosition = 0;
        }


        return targetPosition;
    }  


    public boolean isTargetLeftBranch(String key) {
        Targets currentTarget;
        if ((Targets)data.get(key) != null) {
        currentTarget = (Targets)data.get(key);
        } else {
        currentTarget = Targets.NONE;
        }
        boolean isTargetLeft;
        switch(currentTarget) {
            case A1:
            isTargetLeft = true;
            break;
            case A2:
            isTargetLeft = true;
            break;
            case A3:
            isTargetLeft = true;
            break;
            case A4:
            isTargetLeft = true;
            break;
            case C1:
            isTargetLeft = true;
            break;
            case C2:
            isTargetLeft = true;
            break;
            case C3:
            isTargetLeft = true;
            break;
            case C4:
            isTargetLeft = true;
            break;
            case E1:
            isTargetLeft = true;
            break;
            case E2:
            isTargetLeft = true;
            break;
            case E3:
            isTargetLeft = true;
            break;
            case E4:
            isTargetLeft = true;
            break;
            case G1:
            isTargetLeft = true;
            break;
            case G2:
            isTargetLeft = true;
            break;
            case G3:
            isTargetLeft = true;
            break;
            case G4:
            isTargetLeft = true;
            break;
            case I1:
            isTargetLeft = true;
            break;
            case I2:
            isTargetLeft = true;
            break;
            case I3:
            isTargetLeft = true;
            break;
            case I4:
            isTargetLeft = true;
            break;
            case K1:
            isTargetLeft = true;
            break;
            case K2:
            isTargetLeft = true;
            break;
            case K3:
            isTargetLeft = true;
            break;
            case K4:
            isTargetLeft = true;
            break;
            default:
            isTargetLeft = false;
            break;
        }
        return isTargetLeft;
    }

    public Rotation2d getTargetRotation(String key) {
        Targets currentTarget;
        if ((Targets)data.get(key) != null) {
        currentTarget = (Targets)data.get(key);
        } else {
        currentTarget = Targets.NONE;
        }
        Rotation2d targetRotation = new Rotation2d();
        switch (currentTarget) {
            case A1:
            targetRotation = Constants.reefBranchA.getRotation();
            break;
            case A2:
            targetRotation = Constants.reefBranchA.getRotation();
            break;
            case A3:
            targetRotation = Constants.reefBranchA.getRotation();
            break;
            case A4:
            targetRotation = Constants.reefBranchA.getRotation();
            break;
            case B1:
            targetRotation = Constants.reefBranchB.getRotation();
            break;
            case B2:
            targetRotation = Constants.reefBranchB.getRotation();
            break;
            case B3:
            targetRotation = Constants.reefBranchB.getRotation();
            break;
            case B4:
            targetRotation = Constants.reefBranchB.getRotation();
            break;
            case C1:
            targetRotation = Constants.reefBranchC.getRotation();
            break;
            case C2:
            targetRotation = Constants.reefBranchC.getRotation();
            break;
            case C3:
            targetRotation = Constants.reefBranchC.getRotation();
            break;
            case C4:
            targetRotation = Constants.reefBranchC.getRotation();
            break;
            case D1:
            targetRotation = Constants.reefBranchD.getRotation();
            break;
            case D2:
            targetRotation = Constants.reefBranchD.getRotation();
            break;
            case D3:
            targetRotation = Constants.reefBranchD.getRotation();
            break;
            case D4:
            targetRotation = Constants.reefBranchD.getRotation();
            break;
            case E1:
            targetRotation = Constants.reefBranchE.getRotation();
            break;
            case E2:
            targetRotation = Constants.reefBranchE.getRotation();
            break;
            case E3:
            targetRotation = Constants.reefBranchE.getRotation();
            break;
            case E4:
            targetRotation = Constants.reefBranchE.getRotation();
            break;
            case F1:
            targetRotation = Constants.reefBranchF.getRotation();
            break;
            case F2:
            targetRotation = Constants.reefBranchF.getRotation();
            break;
            case F3:
            targetRotation = Constants.reefBranchF.getRotation();
            break;
            case F4:
            targetRotation = Constants.reefBranchF.getRotation();
            break;
            case G1:
            targetRotation = Constants.reefBranchG.getRotation();
            break;
            case G2:
            targetRotation = Constants.reefBranchG.getRotation();
            break;
            case G3:
            targetRotation = Constants.reefBranchG.getRotation();
            break;
            case G4:
            targetRotation = Constants.reefBranchG.getRotation();
            break;
            case H1:
            targetRotation = Constants.reefBranchH.getRotation();
            break;
            case H2:
            targetRotation = Constants.reefBranchH.getRotation();
            break;
            case H3:
            targetRotation = Constants.reefBranchH.getRotation();
            break;
            case H4:
            targetRotation = Constants.reefBranchH.getRotation();
            break;
            case I1:
            targetRotation = Constants.reefBranchI.getRotation();
            break;
            case I2:
            targetRotation = Constants.reefBranchI.getRotation();
            break;
            case I3:
            targetRotation = Constants.reefBranchI.getRotation();
            break;
            case I4:
            targetRotation = Constants.reefBranchI.getRotation();
            break;
            case J1:
            targetRotation = Constants.reefBranchJ.getRotation();
            break;
            case J2:
            targetRotation = Constants.reefBranchK.getRotation();
            break;
            case J3:
            targetRotation = Constants.reefBranchJ.getRotation();
            break;
            case J4:
            targetRotation = Constants.reefBranchJ.getRotation();
            break;
            case K1:
            targetRotation = Constants.reefBranchK.getRotation();
            break;
            case K2:
            targetRotation = Constants.reefBranchK.getRotation();
            break;
            case K3:
            targetRotation = Constants.reefBranchK.getRotation();
            break;
            case K4:
            targetRotation = Constants.reefBranchK.getRotation();
            break;
            case L1:
            targetRotation = Constants.reefBranchL.getRotation();
            break;
            case L2:
            targetRotation = Constants.reefBranchL.getRotation();
            break;
            case L3:
            targetRotation = Constants.reefBranchL.getRotation();
            break;
            case L4:
            targetRotation = Constants.reefBranchL.getRotation();
            break;
            default:
            targetRotation = Rotation2d.fromRadians(0);
        }
        return targetRotation; 
    }

    public int getTargetID(String key) {
        Targets currentTarget;
        if ((Targets)data.get(key) != null) {
        currentTarget = (Targets)data.get(key);
        } else {
        currentTarget = Targets.NONE;
        }
    int targetID = 0;
    switch (currentTarget) {
        case A1:
        targetID = 18;
        break;
        case A2:
        targetID = 18;
        break;
        case A3:
        targetID = 18;
        break;
        case A4:
        targetID = 18;
        break;
        case B1:
        targetID = 18;
        break;
        case B2:
        targetID = 18;
        break;
        case B3:
        targetID = 18;
        break;
        case B4:
        targetID = 18;
        break;
        case C1:
        targetID = 17;
        break;
        case C2:
        targetID = 17;
        break;
        case C3:
        targetID = 17;
        break;
        case C4:
        targetID = 17;
        break;
        case D1:
        targetID = 17;
        break;
        case D2:
        targetID = 17;
        break;
        case D3:
        targetID = 17;
        break;
        case D4:
        targetID = 17;
        break;
        case E1:
        targetID = 22;
        break;
        case E2:
        targetID = 22;
        break;
        case E3:
        targetID = 22;
        break;
        case E4:
        targetID = 22;
        break;
        case F1:
        targetID = 22;
        break;
        case F2:
        targetID = 22;
        break;
        case F3:
        targetID = 22;
        break;
        case F4:
        targetID = 22;
        break;
        case G1:
        targetID = 21;
        break;
        case G2:
        targetID = 21;
        break;
        case G3:
        targetID = 21;
        break;
        case G4:
        targetID = 21;
        break;
        case H1:
        targetID = 21;
        break;
        case H2:
        targetID = 21;
        break;
        case H3:
        targetID = 21;
        break;
        case H4:
        targetID = 21;
        break;
        case I1:
        targetID = 20;
        break;
        case I2:
        targetID = 20;
        break;
        case I3:
        targetID = 20;
        break;
        case I4:
        targetID = 20;
        break;
        case J1:
        targetID = 20;
        break;
        case J2:
        targetID = 20;
        break;
        case J3:
        targetID = 20;
        break;
        case J4:
        targetID = 20;
        break;
        case K1:
        targetID = 19;
        break;
        case K2:
        targetID = 19;
        break;
        case K3:
        targetID = 19;
        break;
        case K4:
        targetID = 19;
        break;
        case L1:
        targetID = 19;
        break;
        case L2:
        targetID = 19;
        break;
        case L3:
        targetID = 19;
        break;
        case L4:
        targetID = 19;
        break;
        default:
        targetID = 0;
        break;
    }
    return targetID;

    }
}
