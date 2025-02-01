package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;
import java.util.function.Predicate;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;
import frc.robot.lib.BehaviorTree.nodes.DriveHumanPlayerStationRight;
import frc.robot.lib.BehaviorTree.nodes.DriveToIntake;
import frc.robot.lib.BehaviorTree.nodes.DriveToPose;
import frc.robot.lib.BehaviorTree.nodes.DriveToReefBackCenter;
import frc.robot.lib.BehaviorTree.nodes.DriveToTargetPose;
import frc.robot.lib.BehaviorTree.nodes.EmptyNode;
import frc.robot.lib.BehaviorTree.nodes.InfiniteLoopNode;
import frc.robot.lib.BehaviorTree.nodes.SelectorNode;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;
import frc.robot.lib.BehaviorTree.nodes.Subtree;

public class ControlTree {
    Blackboard blackboard;
    InfiniteLoopNode tree;
    BehaviorTreeCommand command;
    Predicate<Blackboard> stopCondition;
    //scoring tree
    Subtree scoringTree;
    //check for target: child nodes - selectornode(check for gamepiece) // Empty leaf node
    SelectorNode findTarget;
    //check for gamepiece: child nodes - sequence node to score // intake subtree
    SelectorNode findPiece;
    //intaking tree for coral
    SequenceNode intakeCoral;
    //intaking nodes for Algae
    SequenceNode intakeAlgae;
    //Sequence node to run to poses and do subsystems and stuff
    SequenceNode scoringNodes;
    //Selector node to check if it has a coral
    SelectorNode hasCoral;
    //Selector node to check if it has Algae
    SelectorNode hasAlgae;

    BehaviorTreeNode emptyleafnode;


    public ControlTree(Blackboard blackboard) {
        this.blackboard = blackboard;
        this.scoringTree = new Subtree(blackboard);
        this.findTarget = new SelectorNode(blackboard);
        this.findPiece = new SelectorNode(blackboard);
        this.hasCoral = new SelectorNode(blackboard);
        this.hasAlgae = new SelectorNode(blackboard);
        this.scoringNodes = new SequenceNode(blackboard);
        this.intakeCoral = new SequenceNode(blackboard);
        this.intakeAlgae = new SequenceNode(blackboard);

        blackboard.set("treeOn", true);


        ((SelectorNode)findTarget).addChild(findPiece, (Blackboard bb) -> bb.getBoolean("hasTarget"));
        ((SelectorNode)findTarget).addChild(new EmptyNode(blackboard), (Blackboard bb) -> !bb.getBoolean("hasTarget"));

        ((SelectorNode)findPiece).addChild(hasAlgae, (Blackboard bb) -> bb.isTargetAlgae("target"));
        ((SelectorNode)findPiece).addChild(hasCoral, (Blackboard bb) -> !bb.isTargetAlgae("target"));

        ((SelectorNode)hasCoral).addChild(scoringNodes, (Blackboard bb) -> bb.getBoolean("hasCoral"));
        ((SelectorNode)hasCoral).addChild(intakeCoral, (Blackboard bb) -> !bb.getBoolean("hasCoral"));

        ((SelectorNode)hasAlgae).addChild(scoringNodes, (Blackboard bb) -> bb.getBoolean("hasAlgae"));
        ((SelectorNode)hasAlgae).addChild(intakeAlgae, (Blackboard bb) -> !bb.getBoolean("hasAlgae"));

        ((SequenceNode)scoringNodes).addChild(new DriveToTargetPose(blackboard));

        ((Subtree)scoringTree).addChild(findTarget);

        ((SequenceNode)intakeCoral).addChild(new DriveToIntake(blackboard));
        ((SequenceNode)intakeAlgae).addChild(new DriveToIntake(blackboard));


        this.tree = new InfiniteLoopNode(blackboard, scoringTree, (Blackboard bb) -> !bb.getBoolean("treeOn"));
    

        this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
