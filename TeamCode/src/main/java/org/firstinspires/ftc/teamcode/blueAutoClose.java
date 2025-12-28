package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "blueAutoClose", group = "Autonomous")
@Configurable
public class blueAutoClose extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private Timer pathTimer, opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        super.start();
        pathTimer.resetTimer();
        telemetry.addData("Auto Start", "Following shootPreload");
        telemetry.update();
        panelsTelemetry.debug("Next Path", "shootPreload");
        follower.followPath(paths.shootPreload, true);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
        panelsTelemetry.debug("Timer (s)", pathTimer.getElapsedTimeSeconds());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain shootPreload, lineupField1, intakeField1, shootField1,
                lineupField2, intakeField2, shootField2, strafeOffLine;

        public Paths(Follower follower) {
            shootPreload = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(24.772, 125.069), new Pose(55.586, 94.255)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build();

            lineupField1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.586, 94.255), new Pose(44.509, 85.796)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            intakeField1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(44.509, 85.796), new Pose(15.910, 85.997)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootField1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.910, 85.997), new Pose(55.586, 94.255)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            lineupField2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.586, 94.255), new Pose(40.683, 60.420)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            intakeField2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(40.683, 60.420), new Pose(15.105, 60.218)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootField2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(15.105, 60.218), new Pose(55.787, 94.456)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            strafeOffLine = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(55.787, 94.456), new Pose(56.996, 115.401)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if (pathTimer.getElapsedTimeSeconds() > 8.0) {
                    // TODO: Shoot preload
                }
                if (pathTimer.getElapsedTimeSeconds() > 13.0) {
                    // TODO: Stop shooting
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "lineupField1");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "lineupField1");
                    follower.followPath(paths.lineupField1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "intakeField1");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "intakeField1");
                    follower.followPath(paths.intakeField1);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "shootField1");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "shootField1");
                    follower.followPath(paths.shootField1);
                    pathTimer.resetTimer();
                }
                if (pathTimer.getElapsedTimeSeconds() > 8.0) {
                    // TODO: Shoot first field sample
                }
                if (pathTimer.getElapsedTimeSeconds() > 13.0) {
                    // TODO: Stop shooting
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "lineupField2");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "lineupField2");
                    follower.followPath(paths.lineupField2);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "intakeField2");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "intakeField2");
                    follower.followPath(paths.intakeField2);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "shootField2");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "shootField2");
                    follower.followPath(paths.shootField2);
                    pathTimer.resetTimer();
                }
                if (pathTimer.getElapsedTimeSeconds() > 8.0) {
                    // TODO: Shoot second field sample
                }
                if (pathTimer.getElapsedTimeSeconds() > 13.0) {
                    // TODO: Stop shooting
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    telemetry.addData("Next Path", "strafeOffLine (Park)");
                    telemetry.update();
                    panelsTelemetry.debug("Next Path", "strafeOffLine (Park)");
                    follower.followPath(paths.strafeOffLine);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    telemetry.addData("Auto Complete", "All paths finished");
                    telemetry.update();
                    requestOpModeStop();
                }
                break;
        }
        return pathState;
    }

    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
