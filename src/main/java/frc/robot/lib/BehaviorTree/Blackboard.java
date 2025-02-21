package frc.robot.lib.BehaviorTree;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
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

    public boolean isPlayerStationLeft() {
        if (data.get("playerStation") == "left") {
            return true;
        } else {
            return false;
        }
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
                default:
                    targetPose = null;
                    break;
            }
        } else {
            targetPose = null;
        }
        return targetPose;
    }

    public boolean isTargetLeftBranch(String key) {
        Targets currentTarget = (Targets)data.get(key);
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
        Targets currentTarget = (Targets)data.get(key);
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
}