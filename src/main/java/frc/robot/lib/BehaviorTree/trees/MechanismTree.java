package frc.robot.lib.BehaviorTree.trees;

import java.util.function.Predicate;

import frc.robot.lib.BehaviorTree.BehaviorTreeCommand;
import frc.robot.lib.BehaviorTree.Blackboard;
import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;
import frc.robot.lib.BehaviorTree.nodes.DriveToIntake;
import frc.robot.lib.BehaviorTree.nodes.ElevatorPose;
import frc.robot.lib.BehaviorTree.nodes.EmptyNode;
import frc.robot.lib.BehaviorTree.nodes.InfiniteLoopNode;
import frc.robot.lib.BehaviorTree.nodes.IntakePivot;
import frc.robot.lib.BehaviorTree.nodes.IntakeRoller;
import frc.robot.lib.BehaviorTree.nodes.ScorerPivot;
import frc.robot.lib.BehaviorTree.nodes.ScorerRoller;
import frc.robot.lib.BehaviorTree.nodes.SelectorNode;
import frc.robot.lib.BehaviorTree.nodes.SequenceNode;
import frc.robot.lib.BehaviorTree.nodes.Subtree;


public class MechanismTree {
    Blackboard blackboard;
    InfiniteLoopNode tree;
    //SequenceNode tree;
    BehaviorTreeCommand command;
    Predicate<Blackboard> stopCondition;
    //scoring tree
    Subtree subsystemTree;
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
        this.subsystemTree = new Subtree(blackboard);

        
        this.scoringNodes = new SequenceNode(blackboard);


        blackboard.set("treeOn", true);

         ((Subtree)subsystemTree).addChild(findTarget);
         
        
         this.tree = new InfiniteLoopNode(blackboard, subsystemTree, (Blackboard bb) -> !bb.getBoolean("treeOn"));
         this.subsystemTree.addChild(new ScorerPivot(blackboard));
         this.subsystemTree.addChild(new ScorerRoller(blackboard));
         this.subsystemTree.addChild(new IntakeRoller(blackboard));
         this.subsystemTree.addChild(new IntakePivot(blackboard));
         this.subsystemTree.addChild(new ElevatorPose(blackboard));

 

         this.command = new BehaviorTreeCommand(tree);


    }

    public BehaviorTreeCommand execute() {
        return command;
    }
    
}
