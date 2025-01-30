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

    public Pose2d getTargetPose(String key) {
        Targets currentTarget = (Targets)data.get(key);
        Pose2d targetPose = new Pose2d();
        switch(currentTarget) {
            case PROCESSOR:
            targetPose = Constants.reefBranchH;
            break;
            case NET:
            targetPose = Constants.humanPlayerStationLeft;
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
        }
        
        return targetPose;
    }
}
