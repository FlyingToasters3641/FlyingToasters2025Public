package frc.robot.lib.BehaviorTree.trees;

import java.util.function.Predicate;

import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;
import frc.robot.lib.BehaviorTree.nodes.EmptyNode;
import frc.robot.lib.BehaviorTree.nodes.InfiniteLoopNode;
import frc.robot.lib.BehaviorTree.nodes.SelectorNode;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;
import frc.robot.lib.BehaviorTree.nodes.Subtree;


public class MechanismTree {
    Blackboard blackboard;
    SequenceNode tree;
    //SequenceNode tree;
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
    //Empty leaf node to cycle through the tree
    BehaviorTreeNode emptyleafnode;
    //
    SequenceNode hasTargetSequence;




    public MechanismTree(Blackboard blackboard) {
        this.blackboard = blackboard;
        this.scoringTree = new Subtree(blackboard);
        this.findTarget = new SelectorNode(blackboard);
        this.findPiece = new SelectorNode(blackboard);
        this.hasCoral = new SelectorNode(blackboard);
        this.hasAlgae = new SelectorNode(blackboard);
        this.scoringNodes = new SequenceNode(blackboard);
        this.intakeCoral = new SequenceNode(blackboard);
        this.intakeAlgae = new SequenceNode(blackboard);
        this.hasTargetSequence = new SequenceNode(blackboard);

        blackboard.set("treeOn", true);

        // ((SelectorNode)findTarget).addChild(findPiece, (Blackboard bb) -> bb.getBoolean("hasTarget"));
        // ((SelectorNode)findTarget).addChild(new EmptyNode(blackboard), (Blackboard bb) -> !bb.getBoolean("hasTarget"));

        // ((SelectorNode)findPiece).addChild(hasAlgae, (Blackboard bb) -> bb.isTargetAlgae("target"));
        // ((SelectorNode)findPiece).addChild(hasCoral, (Blackboard bb) -> !bb.isTargetAlgae("target"));

        // ((SelectorNode)hasCoral).addChild(intakeCoral, (Blackboard bb) -> !bb.getBoolean("hasCoral"));

        // ((SelectorNode)hasAlgae).addChild(intakeAlgae, (Blackboard bb) -> !bb.getBoolean("hasAlgae"));

        // ((Subtree)scoringTree).addChild(findTarget);

        // this.tree = new InfiniteLoopNode(blackboard, scoringTree, (Blackboard bb) -> !bb.getBoolean("treeOn"));
        // this.tree = new SequenceNode(blackboard);
        // ((SequenceNode)tree).addChild(scoringTree);
    

        // this.command = new BehaviorTreeCommand(tree);
    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
