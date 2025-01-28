package frc.robot.lib.BehaviorTree.trees;

import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.DriveHumanPlayerStationRight;
import frc.robot.lib.BehaviorTree.nodes.DriveToReefBackCenter;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;

public class ExampleTree {
    Blackboard blackboard;
    SequenceNode tree;
    BehaviorTreeCommand command;

    public ExampleTree(Blackboard blackboard) {
        this.blackboard = blackboard;
        this.tree = new SequenceNode(blackboard);
        ((SequenceNode) tree).addChild(new DriveHumanPlayerStationRight(blackboard));
        ((SequenceNode) tree).addChild(new DriveToReefBackCenter(blackboard));
        this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
