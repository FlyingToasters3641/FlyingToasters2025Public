package frc.robot.lib.BehaviorTree;

import java.util.concurrent.ConcurrentHashMap;

public class Blackboard {
    private final ConcurrentHashMap<String, Object> data = new ConcurrentHashMap<>();

    public void set(String key, Object value) {
        data.put(key, value);
    }

    public Object get(String key) {
        return data.get(key);
    }

    public boolean containsKey(String key) {
        return data.containsKey(key);
    }
}
