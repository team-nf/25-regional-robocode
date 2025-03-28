public class TrajectoryFollower {
    private Trajectory trajectory;
    private double startTime;

    public TrajectoryFollower(Trajectory trajectory) {
        this.trajectory = trajectory;
        this.startTime = Timer.getFPGATimestamp();  // WPILib timer
    }

    public TrajectoryPoint getCurrentSetpoint() {
        double currentTime = Timer.getFPGATimestamp() - startTime;
        return trajectory.sample(currentTime);
    }

    public boolean isFinished() {
        double currentTime = Timer.getFPGATimestamp() - startTime;
        return currentTime >= trajectory.sample(currentTime).time;
    }
}
