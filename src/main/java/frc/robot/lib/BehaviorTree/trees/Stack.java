package frc.robot.lib.BehaviorTree.trees;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.BehaviorTree.Blackboard;

public class Stack {
    
    ArrayList<Targets> stack = new ArrayList<Targets>();
    Blackboard blackboard;

    public Stack(Blackboard blackboard) {
    this.blackboard = blackboard;

    }

    public void add(Targets item) {
        stack.add(item);
    }

    public Targets get(int num) {
        return stack.get(num);
    }

    public void remove(int num) {
        stack.remove(num);
    }

    public Targets getLastElement() {
        if (stack == null || stack.isEmpty()) {
            return null;
        } else {
        int num = stack.size() - 1;
        return stack.get(num);
        }
    }

    public void removeLastElement() {
        if (stack == null || stack.isEmpty()) {

        } else {
        int num = stack.size() - 1;
        stack.remove(num);
        }
    }

    public boolean isNotEmpty() {
        if (stack.isEmpty()) {
            return false;
        } else {
            return true;
        }
    }

    public boolean isEmpty() {
        if (stack.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    public ArrayList<Targets> getList() {
        if (isNotEmpty()) {
            return stack;
        } else {
            return null;
        }
    }
}
