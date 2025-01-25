package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.DriveHumanPlayerStationRight;
import frc.robot.lib.BehaviorTree.nodes.DriveToPose;
import frc.robot.lib.BehaviorTree.nodes.DriveToReefBackCenter;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;

public class DrivingTree {
    Blackboard blackboard;
    SequenceNode tree;
    BehaviorTreeCommand command;

    public DrivingTree(Blackboard blackboard, ArrayList<Pose2d> poses) {
        this.blackboard = blackboard;
        this.tree = new SequenceNode(blackboard);
        for (int i = 0; i < poses.size(); i++) {
            ((SequenceNode) tree).addChild(new DriveToPose(blackboard, poses.get(i)));
        }
        this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
