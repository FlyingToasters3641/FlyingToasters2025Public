package frc.robot.lib.BehaviorTree;

import java.util.concurrent.ConcurrentHashMap;

import edu.wpi.first.math.geometry.Pose2d;
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

    public boolean containsKey(String key) {
        return data.containsKey(key);
    }

    public boolean getBoolean(String key) {
        return (boolean) data.get(key);
    }

    public boolean isTargetAlgae(String key) {
        Targets currentTarget = (Targets)data.get(key);
        boolean Algae;
        switch(currentTarget) {
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
        return Algae;

    }

    public Pose2d getTargetPose(String key) {
        Targets currentTarget = (Targets)data.get(key);
        Pose2d targetPose = new Pose2d();
        switch(currentTarget) {
            case PROCESSOR:
            targetPose = Constants.processorPose;
            break;
            case NET:
            targetPose = Constants.netPose;
            break;
            case AL1:
            targetPose = Constants.reefBranchA;
            break;
            case AL2: 
            targetPose = Constants.reefBranchA;
            break;
            case AL3:
            targetPose = Constants.reefBranchA;
            break;
            case AL4:
            targetPose = Constants.reefBranchA;
            break;
            case BL1:
            targetPose = Constants.reefBranchB;
            break;
            case BL2:
            targetPose = Constants.reefBranchB;
            break;
            case BL3:
            targetPose = Constants.reefBranchB;
            break;
            case BL4:
            targetPose = Constants.reefBranchB;
            break;
            case CL1:
            targetPose = Constants.reefBranchC;
            break;
            case CL2:
            targetPose = Constants.reefBranchC;
            break;
            case CL3:
            targetPose = Constants.reefBranchC;
            break;
            case CL4:
            targetPose = Constants.reefBranchC;
            break;
            case DL1:
            targetPose = Constants.reefBranchD;
            break;
            case DL2:
            targetPose = Constants.reefBranchD;
            break;
            case DL3:
            targetPose = Constants.reefBranchD;
            break;
            case DL4:
            targetPose = Constants.reefBranchD;
            break;
            case EL1:
            targetPose = Constants.reefBranchE;
            break;
            case EL2:
            targetPose = Constants.reefBranchE;
            break;
            case EL3:
            targetPose = Constants.reefBranchE;
            break;
            case EL4:
            targetPose = Constants.reefBranchE;
            break;
            case FL1:
            targetPose = Constants.reefBranchF;
            break;
            case FL2:
            targetPose = Constants.reefBranchF;
            break;
            case FL3:
            targetPose = Constants.reefBranchF;
            break;
            case FL4:
            targetPose = Constants.reefBranchF;
            break;
            case GL1:
            targetPose = Constants.reefBranchG;
            break;
            case GL2:
            targetPose = Constants.reefBranchG;
            break;
            case GL3:
            targetPose = Constants.reefBranchG;
            break;
            case GL4:
            targetPose = Constants.reefBranchG;
            break;
            case HL1:
            targetPose = Constants.reefBranchH;
            break;
            case HL2:
            targetPose = Constants.reefBranchH;
            break;
            case HL3:
            targetPose = Constants.reefBranchH;
            break;
            case HL4:
            targetPose = Constants.reefBranchH;
            break;
            case IL1:
            targetPose = Constants.reefBranchI;
            break;
            case IL2:
            targetPose = Constants.reefBranchI;
            break;
            case IL3:
            targetPose = Constants.reefBranchI;
            break;
            case IL4:
            targetPose = Constants.reefBranchI;
            break;
            case JL1:
            targetPose = Constants.reefBranchJ;
            break;
            case JL2:
            targetPose = Constants.reefBranchJ;
            break;
            case JL3:
            targetPose = Constants.reefBranchJ;
            break;
            case JL4:
            targetPose = Constants.reefBranchJ;
            break;
            case KL1:
            targetPose = Constants.reefBranchK;
            break;
            case KL2:
            targetPose = Constants.reefBranchK;
            break;
            case KL3:
            targetPose = Constants.reefBranchK;
            break;
            case KL4:
            targetPose = Constants.reefBranchK;
            break;
            case LL1:
            targetPose = Constants.reefBranchL;
            break;
            case LL2:
            targetPose = Constants.reefBranchL;
            break;
            case LL3:
            targetPose = Constants.reefBranchL;
            break;
            case LL4:
            targetPose = Constants.reefBranchL;
            break;
        }
        
        return targetPose;
    }
}
