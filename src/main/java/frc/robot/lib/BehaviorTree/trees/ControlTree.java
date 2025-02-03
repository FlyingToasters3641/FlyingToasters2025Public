package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.DriveHumanPlayerStationRight;
import frc.robot.lib.BehaviorTree.nodes.DriveToPose;
import frc.robot.lib.BehaviorTree.nodes.DriveToReefBackCenter;
import frc.robot.lib.BehaviorTree.nodes.InfiniteLoopNode;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;

public class ControlTree {
    Blackboard blackboard;
    SequenceNode tree;
    BehaviorTreeCommand command;
    Predicate<Blackboard> stopCondition;

    public ControlTree(Blackboard blackboard) {
        this.blackboard = blackboard;
        //this.tree = new InfiniteLoopNode(blackboard, new SequenceNode(blackboard), stopCondition);
        this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
