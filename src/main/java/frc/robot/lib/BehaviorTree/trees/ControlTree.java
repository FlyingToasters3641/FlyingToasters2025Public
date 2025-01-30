package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;
import frc.robot.lib.BehaviorTree.nodes.DriveHumanPlayerStationRight;
import frc.robot.lib.BehaviorTree.nodes.DriveToPose;
import frc.robot.lib.BehaviorTree.nodes.DriveToReefBackCenter;
import frc.robot.lib.BehaviorTree.nodes.DriveToTargetPose;
import frc.robot.lib.BehaviorTree.nodes.InfiniteLoopNode;
import frc.robot.lib.BehaviorTree.nodes.SelectorNode;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;
import frc.robot.lib.BehaviorTree.nodes.Subtree;

public class ControlTree {
    Blackboard blackboard;
    SequenceNode tree;
    BehaviorTreeCommand command;
    Predicate<Blackboard> stopCondition;
    //scoring tree
    Subtree scoringTree;
    //check for target: child nodes - selectornode(check for gamepiece) // Empty leaf node
    SelectorNode selectorNode1;
    //check for gamepiece: child nodes - sequence node to score // intake subtree
    SelectorNode selectorNode2;
    //intaking tree
    Subtree intakeTree;
    //Sequence node to run to poses and do subsystems and stuff
    SequenceNode scoringNodes;

    BehaviorTreeNode emptyleafnode;


    public ControlTree(Blackboard blackboard) {
        this.blackboard = blackboard;
        this.scoringTree = new Subtree(blackboard);
        this.selectorNode1 = new SelectorNode(blackboard);
        this.selectorNode2 = new SelectorNode(blackboard);
        this.scoringNodes = new SequenceNode(blackboard);
        this.intakeTree = new Subtree(blackboard);

        blackboard.set("hasPiece", true);
        blackboard.set("hasAlgae", true);
        blackboard.set("target", Targets.AL1);


        ((SelectorNode)selectorNode1).addChild(selectorNode2, (Blackboard bb) -> bb.getBoolean("hasPiece"));
        ((SelectorNode)selectorNode1).addChild(emptyleafnode, (Blackboard bb) -> !bb.getBoolean("hasPiece"));
        //doesn't work for both gamepieces yet
        ((SelectorNode)selectorNode2).addChild(scoringNodes, (Blackboard bb) -> bb.getBoolean("hasAlgae"));
        ((SelectorNode)selectorNode2).addChild(emptyleafnode, (Blackboard bb) -> !bb.getBoolean("hasAlgae"));

        ((SequenceNode)scoringNodes).addChild(new DriveToTargetPose(blackboard));
        ((SequenceNode)scoringNodes).addChild(emptyleafnode);

        ((Subtree)scoringTree).addChild(selectorNode1);


        this.tree = new SequenceNode(blackboard);
        tree.addChild(scoringTree);
    

        this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
