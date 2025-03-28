
package frc.robot.util;

import java.io.*;
import java.util.*;
import com.fasterxml.jackson.databind.ObjectMapper;

public class PythonTrajectory {

    public static ArmTraj generate(double[] fromPose, double[] toPose) throws IOException {
        File tempFile = File.createTempFile("temp_trajectory_", ".json");
        tempFile.deleteOnExit();

        Map<String, Object> poseMap = new HashMap<>();
        List<double[]> poses = new ArrayList<>();
        poses.add(fromPose);
        poses.add(toPose);
        poseMap.put("poses", poses);

        ObjectMapper mapper = new ObjectMapper();
        mapper.writeValue(tempFile, poseMap);

        ProcessBuilder pb = new ProcessBuilder(
            "python3", "generate_trajectory.py",
            "--poses", tempFile.getAbsolutePath(),
            "--out", "trajectory.json"
        );
        pb.redirectErrorStream(true);
        Process proc = pb.start();

        BufferedReader reader = new BufferedReader(new InputStreamReader(proc.getInputStream()));
        String line;
        while ((line = reader.readLine()) != null) {
            System.out.println("[Python] " + line);
        }

        try {
            proc.waitFor();
        } catch (InterruptedException e) {
            throw new RuntimeException("Python subprocess interrupted", e);
        }

        LoadTrajectory.init("trajectory.json");
        return LoadTrajectory.trajectory;
    }
}
