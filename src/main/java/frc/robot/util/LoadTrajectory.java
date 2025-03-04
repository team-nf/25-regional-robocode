package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.Map;

public class LoadTrajectory {
    public static ArmTraj trajectory;

    public static void init() {
        ObjectMapper objectMapper = new ObjectMapper();

        try {
            // Read the JSON file into a Map
            Map<String, Map<String, Double>> trajectoryData = objectMapper.readValue(new File("trajectory.json"), Map.class);

            // Extract x and y positions from the JSON structure
            Map<String, Double> xData = trajectoryData.get("theta");
            Map<String, Double> phiData = trajectoryData.get("phi");
            Map<String, Double> yData = trajectoryData.get("h");

            // Create arrays to hold x and y values (size = 100)
            double[] xValues = new double[xData.size()];
            double[] yValues = new double[yData.size()];
            double[] phiValues = new double[phiData.size()];

            // Populate x and y arrays from the JSON data
            int index = 0;
            for (String key : xData.keySet()) {
                xValues[index] = xData.get(key);
                yValues[index] = yData.get(key);
                phiValues[index] = phiData.get(key);
                index++;
            }

            // Generate a name for the trajectory, can be changed based on your logic
            String trajectoryName = "trajectory";  // Change the name as needed

            // Create an ArmTraj object to represent the full trajectory
            trajectory = new ArmTraj(trajectoryName, xValues, phiValues ,yValues);

            // Print the trajectory
            System.out.println(trajectory);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
