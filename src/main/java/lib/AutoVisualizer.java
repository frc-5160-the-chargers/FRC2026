package lib;

import choreo.auto.AutoTrajectory;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.List;

public class AutoVisualizer {
    private static final double END_DELAY_SECS = 0.5, START_DELAY_SECS = 0.5;

    private final Field2d field = new Field2d();
    private final double dt;
    private final List<Trajectory<? extends TrajectorySample<?>>> trajectories = new ArrayList<>();
    private double totalTimeOfAuto = 0.0;
    private double currentTime = 0.0;

    public AutoVisualizer(String name, double movementSpeedMultiplier) {
        this.dt = movementSpeedMultiplier * 0.02;
        SmartDashboard.putData(name, field);
    }

    public AutoVisualizer(String name) {
        this(name, 1.0);
    }

    public void setAutoSequence(AutoTrajectory... trajectories) {
        setAutoSequence(List.of(trajectories));
    }

    public void setAutoSequence(List<AutoTrajectory> trajectories) {
        if (trajectories.isEmpty()) return;
        var startPoseViz = this.field.getObject("StartingPose");
        trajectories.get(0).getInitialPose().ifPresent(startPoseViz::setPose);
        for (var previousTraj: this.trajectories) {
            this.field.getObject("Trajectory_" + previousTraj.name()).setPoses(List.of());
        }
        this.trajectories.clear();
        this.totalTimeOfAuto = 0.0;
        this.currentTime = -START_DELAY_SECS;
        for (var traj: trajectories) {
            var rawTraj = traj.getRawTrajectory();
            var poses = rawTraj.samples().stream().map(TrajectorySample::getPose).toList();
            this.trajectories.add(rawTraj);
            this.totalTimeOfAuto += rawTraj.getTotalTime();
            this.field.getObject("Trajectory_" + rawTraj.name()).setPoses(poses);
        }
    }

    public void periodic() {
        if (DriverStation.isEnabled() || trajectories.isEmpty()) {
            field.setRobotPose(Pose2d.kZero);
            return;
        }
        double timeIntoNextTraj = Math.max(currentTime, 0);
        for (var traj: trajectories) {
            if (timeIntoNextTraj > traj.getTotalTime()) {
                timeIntoNextTraj -= traj.getTotalTime();
                continue;
            }
            var sample = traj.sampleAt(timeIntoNextTraj, false);
            field.setRobotPose(sample.isPresent() ? sample.get().getPose() : Pose2d.kZero);
            break;
        }
        currentTime += dt;
        if (currentTime > totalTimeOfAuto + END_DELAY_SECS) {
            currentTime = -START_DELAY_SECS;
        }
    }
}
