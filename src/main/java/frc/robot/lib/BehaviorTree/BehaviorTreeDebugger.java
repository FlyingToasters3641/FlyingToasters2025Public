package frc.robot.lib.BehaviorTree;

import java.util.LinkedHashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import frc.robot.lib.BehaviorTree.nodes.BehaviorTreeNode;

public class BehaviorTreeDebugger {
    private static BehaviorTreeDebugger instance;
    private final Map<String, ExecutionStatus> nodeStatuses = new LinkedHashMap<>();

    private boolean enableLogging = true;

    private BehaviorTreeDebugger() {
    }

    public static BehaviorTreeDebugger getInstance() {
        if (instance == null) {
            instance = new BehaviorTreeDebugger();
        }
        return instance;
    }

    public void logNodeExecution(BehaviorTreeNode node, ExecutionStatus status, BehaviorTreeNode parent) {
        if (enableLogging) {
            String nodeName = (parent == null) ? node.getClass().getSimpleName() :
                parent.getClass().getSimpleName() + "/" + node.getClass().getSimpleName();
            Logger.recordOutput("BehaviorTree/NodeExecution/" + nodeName, status);
            System.out.println("[BehaviorTree] " + nodeName + " | Status: " + status);
        }
        nodeStatuses.put(node.getClass().getSimpleName(), status);
    }

    public void enableLogging(boolean enable) {
        this.enableLogging = enable;
    }

    public void reset() {
        nodeStatuses.clear();
    }

    public void printTreeSummary() {
        System.out.println("===== Behavior Tree Execution Summary =====");
        for (Map.Entry<String, ExecutionStatus> entry : nodeStatuses.entrySet()) {
            System.out.println(entry.getKey() + " -> " + entry.getValue());
        }
    }
}
