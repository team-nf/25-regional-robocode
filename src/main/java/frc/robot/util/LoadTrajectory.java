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
            Map<String, Double> thetaData = trajectoryData.get("theta");
            Map<String, Double> hData = trajectoryData.get("h");
            Map<String, Double> phiData = trajectoryData.get("phi");
            Map<String, Double> xData = trajectoryData.get("x");
            Map<String, Double> yData = trajectoryData.get("y");

            // Create arrays to hold x and y values (size = 100)
            double[] xValues = new double[xData.size()];
            double[] yValues = new double[yData.size()];
            double[] thetaValues = new double[thetaData.size()];
            double[] hValues = new double[hData.size()];
            double[] phiValues = new double[phiData.size()];

            // Populate x and y arrays from the JSON data
            int index = 0;
            for (String key : xData.keySet()) {
                thetaValues[index] = thetaData.get(key);
                hValues[index] = hData.get(key);
                phiValues[index] = phiData.get(key);
                xValues[index] = xData.get(key);
                yValues[index] = yData.get(key);
                index++;
            }

            // Generate a name for the trajectory, can be changed based on your logic
            String trajectoryName = "trajectory";  // Change the name as needed

            // Create an ArmTraj object to represent the full trajectory
            trajectory = new ArmTraj(trajectoryName, thetaValues, phiValues ,hValues, xValues, yValues);

            // Print the trajectory
            System.out.println(trajectory);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
