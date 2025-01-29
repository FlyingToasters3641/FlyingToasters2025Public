package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BehaviorTree.Blackboard;

public class Stack {
    
    ArrayList<String> stack = new ArrayList<String>();
    Blackboard blackboard;

    public Stack(Blackboard blackboard) {
    this.blackboard = blackboard;

    }

    public void add(String item) {
        stack.add(item);
    }

    public String get(int num) {
        return stack.get(num);
    }

    public void remove(int num) {
        stack.remove(num);
    }

    public String getLastElement() {
        int num = stack.size() - 1;
        return stack.get(num);
        

    }

}
