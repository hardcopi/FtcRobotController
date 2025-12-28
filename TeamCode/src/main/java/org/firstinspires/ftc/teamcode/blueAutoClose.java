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
@Configurable // Panels
public class blueAutoClose extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class
    private Timer pathTimer, opmodeTimer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build all paths

        // Initialize state machine
        pathState = 0;

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        super.start();
        // Reset the path timer exactly when PLAY is pressed
        pathTimer.resetTimer();
        // Start the first path (shoot preload)
        follower.followPath(paths.shootPreload, true);
    }

    @Override
    public void loop() {
        // TODO: Start intake, shooter, and transfer wheel here if needed at the beginning
        // e.g. robot.intake.start(); robot.shooter.start(); etc.

        follower.update(); // Must be called every loop for Pedro Pathing to drive
        pathState = autonomousPathUpdate(); // Advance state machine

        // Telemetry logging
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
            shootPreload = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(24.772, 125.069), new Pose(55.586, 94.255)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build();

            lineupField1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(55.586, 94.255), new Pose(44.509, 85.796)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            intakeField1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(44.509, 85.796), new Pose(15.910, 85.997)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootField1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(15.910, 85.997), new Pose(55.586, 94.255)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            lineupField2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(55.586, 94.255), new Pose(40.683, 60.420)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(180))
                    .build();

            intakeField2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(40.683, 60.420), new Pose(15.105, 60.218)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootField2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(15.105, 60.218), new Pose(55.787, 94.456)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .build();

            strafeOffLine = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(55.787, 94.456), new Pose(56.996, 115.401)))
                    .setLinearHeadingInterpolation(Math.toRadians(137), Math.toRadians(137))
                    .build();
        }
    }

    /**
     * State machine that controls the autonomous sequence.
     * Returns the current pathState for telemetry.
     */
    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Following shootPreload (started in start())
                // Timed shooting of preload while holding at scoring pose
                if (pathTimer.getElapsedTimeSeconds() > 8.0) {
                    // TODO: Shoot the preload here (e.g. transferWheel.reverse() or servo release)
                }
                if (pathTimer.getElapsedTimeSeconds() > 13.0) {
                    // TODO: Stop shooting mechanism
                    setPathState(1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.lineupField1);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeField1);
                    setPathState(3);
                }
                break;

            case 3: // Back to scoring pose for first field sample
                if (!follower.isBusy()) {
                    follower.followPath(paths.shootField1);
                    pathTimer.resetTimer(); // Reset for next shooting sequence
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
                    follower.followPath(paths.lineupField2);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.intakeField2);
                    setPathState(6);
                }
                break;

            case 6: // Back to scoring pose for second field sample
                if (!follower.isBusy()) {
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
                    follower.followPath(paths.strafeOffLine);
                    setPathState(8);
                }
                break;

            case 8: // Parking complete
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
        return pathState;
    }

    /**
     * Helper method to advance the state machine and reset the timer.
     */
    private void setPathState(int newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }
}
