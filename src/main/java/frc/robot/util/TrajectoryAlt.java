import java.io.*;
import java.util.*;
import org.json.*;

public class Trajectory {
    private List<TrajectoryPoint> points;

    public Trajectory(String filePath) throws IOException {
        points = new ArrayList<>();
        String content = new String(Files.readAllBytes(Paths.get(filePath)));
        JSONArray json = new JSONArray(content);

        for (int i = 0; i < json.length(); i++) {
            JSONObject point = json.getJSONObject(i);
            double time = point.getDouble("time");
            JSONArray posArray = point.getJSONArray("positions");
            JSONArray velArray = point.getJSONArray("velocities");

            double[] pos = new double[posArray.length()];
            double[] vel = new double[velArray.length()];
            for (int j = 0; j < posArray.length(); j++) {
                pos[j] = posArray.getDouble(j);
                vel[j] = velArray.getDouble(j);
            }

            points.add(new TrajectoryPoint(time, pos, vel));
        }
    }

    public TrajectoryPoint sample(double currentTime) {
        if (currentTime <= points.get(0).time) return points.get(0);
        if (currentTime >= points.get(points.size() - 1).time) return points.get(points.size() - 1);

        for (int i = 0; i < points.size() - 1; i++) {
            TrajectoryPoint p0 = points.get(i);
            TrajectoryPoint p1 = points.get(i + 1);
            if (currentTime >= p0.time && currentTime <= p1.time) {
                double t = (currentTime - p0.time) / (p1.time - p0.time);
                double[] interpPos = new double[p0.positions.length];
                double[] interpVel = new double[p0.velocities.length];
                for (int j = 0; j < interpPos.length; j++) {
                    interpPos[j] = p0.positions[j] + t * (p1.positions[j] - p0.positions[j]);
                    interpVel[j] = p0.velocities[j] + t * (p1.velocities[j] - p0.velocities[j]);
                }
                return new TrajectoryPoint(currentTime, interpPos, interpVel);
            }
        }
        return points.get(0); // fallback
    }
}
