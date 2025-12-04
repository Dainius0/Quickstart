package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main autonomous OpMode using Pedro Pathing
 *
 * This class handles:
 * - Initialization of the Follower
 * - State machine execution
 * - Telemetry updates
 */
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private AutoStateMachine stateMachine;
    private org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths paths;
    private ElapsedTime pathTimer;

    // Starting position on the field
    private static final Pose STARTING_POSE = new Pose(72, 8, Math.toRadians(90));

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Create and configure the follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(STARTING_POSE);

        // Initialize paths and state machine
        paths = new org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths(follower);
        stateMachine = new AutoStateMachine(follower, paths);
        pathTimer = new ElapsedTime();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.debug("Starting Pose", String.format("(%.1f, %.1f, %.1f°)",
                STARTING_POSE.getX(),
                STARTING_POSE.getY(),
                Math.toDegrees(STARTING_POSE.getHeading())));
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        stateMachine.start();
        pathTimer.reset();
    }

    @Override
    public void loop() {
        // Update follower - MUST be called every loop
        follower.update();

        // Update state machine
        stateMachine.update();

        // Update telemetry
        updateTelemetry();
    }

    @Override
    public void stop() {
        // Clean up when autonomous ends
        follower.breakFollowing();
    }

    /**
     * Updates telemetry with current robot state
     */
    private void updateTelemetry() {
        Pose currentPose = follower.getPose();

        panelsTelemetry.debug("=== STATE ===", "");
        panelsTelemetry.debug("Current State", stateMachine.getCurrentStateName());
        panelsTelemetry.debug("Is Busy", follower.isBusy());
        panelsTelemetry.debug("Time Elapsed", String.format("%.2fs", pathTimer.seconds()));

        panelsTelemetry.debug("", "");
        panelsTelemetry.debug("=== POSITION ===", "");
        panelsTelemetry.debug("X", String.format("%.2f", currentPose.getX()));
        panelsTelemetry.debug("Y", String.format("%.2f", currentPose.getY()));
        panelsTelemetry.debug("Heading", String.format("%.1f°", Math.toDegrees(currentPose.getHeading())));

        panelsTelemetry.update(telemetry);
    }
}