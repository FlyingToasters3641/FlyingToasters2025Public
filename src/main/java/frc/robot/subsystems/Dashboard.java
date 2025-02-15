package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.BehaviorTree.trees.Stack;
import frc.robot.lib.BehaviorTree.trees.Targets;

public class Dashboard extends SubsystemBase {
    private final Map<EventLoop, Map<Integer, Trigger>> levelCache = new HashMap<>();
    static NetworkTable table = NetworkTableInstance.getDefault().getTable("Dashboard");
    static NetworkTableEntry elevatorControl = table.getEntry("Level");
    static NetworkTableEntry autoControl = table.getEntry("Target");
    static Targets lastTargetValue = Targets.NONE;

    public Trigger getLevel(int level, EventLoop eventLoop) {
        var cache = levelCache.computeIfAbsent(eventLoop, k -> new HashMap<>());

        return cache.computeIfAbsent(level, k -> new Trigger(() -> getLevelValue() == level));
    }

    public Trigger getLevel(int level) {
        return getLevel(level, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger L1() {
        return getLevel(1);
    }

    public Trigger L2() {
        return getLevel(2);
    }

    public Trigger L3() {
        return getLevel(3);
    }

    public Trigger L4() {
        return getLevel(4);
    }

    public Trigger NET() {
        return getLevel(5);
    }

    public Trigger NONE() {
        return getLevel(0);
    }

    public static int getLevelValue() {
        String level = elevatorControl.getString("NONE");
        int levelValue = 0;
        switch (level) {
            case "L1":
                Logger.recordOutput("Dashboard/Level", "L1");
                levelValue = 1;
                break;
            case "L2":
                Logger.recordOutput("Dashboard/Level", "L2");
                levelValue = 2;
                break;
            case "L3":
                Logger.recordOutput("Dashboard/Level", "L3");
                levelValue = 3;
                break;
            case "L4":
                Logger.recordOutput("Dashboard/Level", "L4");
                levelValue = 4;
                break;
            case "NET":
                Logger.recordOutput("Dashboard/Level", "NET");
                levelValue = 5;
                break;
            default:
                Logger.recordOutput("Dashboard/Level", "NONE");
                levelValue = 0;
                break;
        }

        if (levelValue < 0 || levelValue > 5) {
            DriverStation.reportWarning("Dashboard Level not available, check if dashboard in connected: " + levelValue,
                    false);
            return -1;
        } else {
            return levelValue;
        }

    }

    public static void getTargetValue(Stack stack) {
        if (autoControl.exists()) {
            String target = autoControl.getString("NONE");
            Targets targetValue = Targets.valueOf(target);
            if (targetValue != lastTargetValue) {
                stack.removeMatchingElements(targetValue);
                stack.add(targetValue);
                lastTargetValue = targetValue;
            }
        }
    }
}
