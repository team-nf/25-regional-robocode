
package frc.robot.util;

import java.util.*;
import java.io.File;

public class TrajectoryLibrary {
    private static final String BASE_DIR = "presets/";
    private static final Map<String, String> registry = new HashMap<>();

    static {
        registry.put("pickup_to_highscore", "pickup_to_highscore.json");
        registry.put("stow_to_mid", "stow_to_mid.json");
        registry.put("floor_to_shelf", "floor_to_shelf.json");
    }

    public static ArmTraj get(String name) {
        if (!registry.containsKey(name)) {
            throw new IllegalArgumentException("Unknown trajectory: " + name);
        }
        String filename = BASE_DIR + registry.get(name);
        LoadTrajectory.init(filename);
        return LoadTrajectory.trajectory;
    }

    public static Set<String> availableNames() {
        return registry.keySet();
    }
}
