package frc.robot.lib.BehaviorTree;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants;
import frc.robot.lib.BehaviorTree.trees.IntakePivots;
import frc.robot.lib.BehaviorTree.trees.IntakeRollers;
import frc.robot.lib.BehaviorTree.trees.ScorerRollers;
import frc.robot.lib.BehaviorTree.trees.Targets;

public class Blackboard {
    private final ConcurrentHashMap<String, Object> data = new ConcurrentHashMap<>();

    public void set(String key, Object value) {
        data.put(key, value);
    }

    public Object get(String key) {
        return data.get(key);
    }

    public Targets getTarget(String key) {
        return (Targets) data.get(key);
    }

    public boolean containsKey(String key) {
        return data.containsKey(key);
    }

    public boolean getBoolean(String key) {
        if (data.get(key) == null) {
            //TODO: THIS SETS A RANDOM VARIABLE IF NULL: FIX LATER
            return false;
        } else {
        return (boolean) data.get(key);
        }
    }

    public boolean isTargetAlgae(String key) {
        Targets currentTarget = (Targets) data.get(key);
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
        //TODO: THIS SETS A RANDOM VARIABLE IF NULL
        Algae = false;
    }
        return Algae;

    }

    public Pose2d getTargetPose(String key) {
        Targets currentTarget = (Targets) data.get(key);
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
                case TEST2:
                    targetPose = Constants.testPose1;
                    break;
                case NONE:
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
        IntakePivots currentTarget = (IntakePivots) data.get(key);
        double targetAngle = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
                case IN_Algae:
                    targetAngle = Constants.INTAKE_ALGAE_ANGLE;
                case IN_Coral:
                    targetAngle = Constants.INTAKE_CORAL_ANGLE;
                    break;
                case IN_Rest:
                    targetAngle = Constants.INTAKE_REST_ANGLE;
                    break;
                case IN_ClearElevator:
                    targetAngle = Constants.INTAKE_CLEAR_ELEVATOR_ANGLE;
            }
        } else {
            targetAngle = 0;
        }

        return targetAngle;
    }

    public double getTargetIntakeSpeed(String key) {
        IntakeRollers currentTarget = (IntakeRollers) data.get(key);
        double targetSpeed = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
                case IN_Algae:
                    targetSpeed = Constants.INTAKE_ALGAE_ANGLE;
                case IN_Coral:
                    targetSpeed = Constants.INTAKE_CORAL_SPEED;
                    break;
                case IN_OutakeCoral:
                    targetSpeed = Constants.OUTTAKE_ALGAE_SPEED;
                    break;
                case IN_Stop:
                    targetSpeed = Constants.INTAKE_REST_SPEED;
            }
        } else {
            targetSpeed = 0;
        }

        return targetSpeed;
    }  

     public double getTargetScoerSpeed(String key) {
        ScorerRollers currentTarget = (ScorerRollers) data.get(key);
        double targetSpeed = 0;
        if (currentTarget != null) {
            switch (currentTarget) {
                case CS_SCORE_CORAL:
                    targetSpeed = Constants.SCORER_CORAL_SPEED;
                case CS_SCORE_CORAL_L4:
                    targetSpeed = Constants.SCORER_CORAL_L4_SPEED;
                    break;
                case CS_REMOVE_ALGAE:
                    targetSpeed = Constants.SCORER_REMOVE_ALGAE;
                    break;
                case CS_INTAKE_ALGAE:
                    targetSpeed = Constants.SCORER_INTAKE_ALGAE;
                    break;
                
                case CS_INTAKE_CORAL:
                    targetSpeed = Constants.SCORER_INTAKE_CORAL;
                    break;
                case CS_STOP:
                    targetSpeed = Constants.INTAKE_REST_SPEED;
            }
        } else {
            targetSpeed = 0;
        }

        return targetSpeed;
    }  
}
