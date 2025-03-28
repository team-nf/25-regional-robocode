public class TrajectoryPoint {
    public double time;
    public double[] positions;
    public double[] velocities;

    public TrajectoryPoint(double time, double[] positions, double[] velocities) {
        this.time = time;
        this.positions = positions;
        this.velocities = velocities;
    }
}
