
package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.File;
import java.io.IOException;
import java.util.Map;

public class LoadTrajectory {
    public static ArmTraj trajectory;

    public static void init(String path) {
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            Map<String, Map<String, Double>> data = objectMapper.readValue(new File(path), Map.class);
            double[] theta = new double[data.get("theta").size()];
            double[] phi = new double[data.get("phi").size()];
            double[] h = new double[data.get("h").size()];
            double[] x = new double[data.get("x").size()];
            double[] y = new double[data.get("y").size()];

            int i = 0;
            for (String k : data.get("theta").keySet()) {
                theta[i] = data.get("theta").get(k);
                phi[i] = data.get("phi").get(k);
                h[i] = data.get("h").get(k);
                x[i] = data.get("x").get(k);
                y[i] = data.get("y").get(k);
                i++;
            }

            trajectory = new ArmTraj("dynamic", theta, phi, h, x, y);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
