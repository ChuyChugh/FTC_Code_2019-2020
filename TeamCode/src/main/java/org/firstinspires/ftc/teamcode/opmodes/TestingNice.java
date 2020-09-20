package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;

public class TestingNice extends LinearOpMode {
    private Trajectory trajectory;
    public void generateTrajectory() {

        double meterspersecond = 1150 / 60 * 4 * Math.PI * 2.54 / 100;
        Pose2d sideStart = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(180));
        Pose2d scale = new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(-90));
        ArrayList interwaypoints = new ArrayList<Translation2d>();
        interwaypoints.add(new Translation2d(1, 1));
        interwaypoints.add(new Translation2d(1, 1));
        TrajectoryConfig config = new TrajectoryConfig(meterspersecond, meterspersecond);
        config.setReversed(true);
        trajectory = TrajectoryGenerator.generateTrajectory(
                sideStart,
                interwaypoints,
                scale,
                config
        );
    }

    

    @Override
    public void runOpMode() throws InterruptedException {
        generateTrajectory();
        double duration = trajectory.getTotalTimeSeconds();
        Trajectory.State point = trajectory.sample(1.5);
        Pose2d aOrigin = new Pose2d(2,2,Rotation2d.fromDegrees(30));
        Pose2d bOrigin = new Pose2d(2,2,Rotation2d.fromDegrees(180));
        Trajectory aTrajectory = trajectory.relativeTo(bOrigin);
        Transform2d transform = new Pose2d(4,4,Rotation2d.fromDegrees(150)).minus(trajectory.getInitialPose());
        Trajectory newTing = trajectory.transformBy(transform);
    }
}
