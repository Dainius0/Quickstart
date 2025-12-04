package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;

/**
 * CREATE THIS AS A SEPARATE FILE: AutoStateMachine.java
 *
 * State machine that controls the autonomous routine sequence
 *
 * How it works:
 * 1. Each "action" state (PATH_1, PATH_2, etc.) starts a path
 * 2. Immediately transitions to a "waiting" state (+100)
 * 3. Waits until follower.isBusy() returns false
 * 4. Moves to next action state
 *
 * This pattern ensures paths start only once and complete before moving on.
 */
public class AutoStateMachine {

    // State constants
    private static final int STATE_IDLE = 0;
    private static final int STATE_PATH_1 = 1;
    private static final int STATE_PATH_2 = 2;
    private static final int STATE_PATH_3 = 3;
    private static final int STATE_PATH_4 = 4;
    private static final int STATE_FINISHED = 5;

    // Offset for "waiting" substates
    private static final int WAITING_OFFSET = 100;

    private final Follower follower;
    private final org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths paths;
    private int currentState;

    /**
     * Creates a new state machine
     * @param follower The Follower to control
     * @param paths The paths to execute
     */
    public AutoStateMachine(Follower follower, org.firstinspires.ftc.teamcode.pedroPathing.AutoPaths paths) {
        this.follower = follower;
        this.paths = paths;
        this.currentState = STATE_IDLE;
    }

    /**
     * Starts the autonomous routine
     */
    public void start() {
        currentState = STATE_PATH_1;
    }

    /**
     * Updates the state machine - should be called every loop
     */
    public void update() {
        switch (currentState) {
            case STATE_IDLE:
                // Waiting for start
                break;

            // PATH 1: Start and wait for completion
            case STATE_PATH_1:
                follower.followPath(paths.path1);
                currentState = STATE_PATH_1 + WAITING_OFFSET;
                break;

            case STATE_PATH_1 + WAITING_OFFSET:
                if (!follower.isBusy()) {
                    currentState = STATE_PATH_2;
                }
                break;

            // PATH 2: Start and wait for completion
            case STATE_PATH_2:
                follower.followPath(paths.path2);
                currentState = STATE_PATH_2 + WAITING_OFFSET;
                break;

            case STATE_PATH_2 + WAITING_OFFSET:
                if (!follower.isBusy()) {
                    currentState = STATE_PATH_3;
                }
                break;

            // PATH 3: Start and wait for completion
            case STATE_PATH_3:
                follower.followPath(paths.path3);
                currentState = STATE_PATH_3 + WAITING_OFFSET;
                break;

            case STATE_PATH_3 + WAITING_OFFSET:
                if (!follower.isBusy()) {
                    currentState = STATE_PATH_4;
                }
                break;

            // PATH 4: Start and wait for completion
            case STATE_PATH_4:
                follower.followPath(paths.path4);
                currentState = STATE_PATH_4 + WAITING_OFFSET;
                break;

            case STATE_PATH_4 + WAITING_OFFSET:
                if (!follower.isBusy()) {
                    currentState = STATE_FINISHED;
                }
                break;

            case STATE_FINISHED:
                // Autonomous complete - do nothing
                break;

            default:
                // Unknown state - reset to idle
                currentState = STATE_IDLE;
                break;
        }
    }

    /**
     * Gets the current state number
     */
    public int getCurrentState() {
        return currentState;
    }

    /**
     * Gets a human-readable name for the current state
     */
    public String getCurrentStateName() {
        return getStateName(currentState);
    }

    /**
     * Converts a state number to a readable name
     */
    private String getStateName(int state) {
        switch (state) {
            case STATE_IDLE:
                return "IDLE";
            case STATE_PATH_1:
                return "PATH_1_START";
            case STATE_PATH_1 + WAITING_OFFSET:
                return "PATH_1_RUNNING";
            case STATE_PATH_2:
                return "PATH_2_START";
            case STATE_PATH_2 + WAITING_OFFSET:
                return "PATH_2_RUNNING";
            case STATE_PATH_3:
                return "PATH_3_START";
            case STATE_PATH_3 + WAITING_OFFSET:
                return "PATH_3_RUNNING";
            case STATE_PATH_4:
                return "PATH_4_START";
            case STATE_PATH_4 + WAITING_OFFSET:
                return "PATH_4_RUNNING";
            case STATE_FINISHED:
                return "FINISHED";
            default:
                return "UNKNOWN";
        }
    }

    /**
     * Checks if the autonomous routine is complete
     */
    public boolean isFinished() {
        return currentState == STATE_FINISHED;
    }

    /**
     * Example: Adding actions between paths
     *
     * You can add custom states for actions like shooting, intake, etc.
     * between path states:
     */
    /*
    private static final int STATE_SHOOT_1 = 10;
    private static final int STATE_INTAKE = 11;

    // In the update() method:
    case STATE_PATH_1 + WAITING_OFFSET:
        if (!follower.isBusy()) {
            currentState = STATE_SHOOT_1;  // Go to shooting state instead of next path
        }
        break;

    case STATE_SHOOT_1:
        // Start shooting mechanism
        shooter.shoot();
        currentState = STATE_SHOOT_1 + WAITING_OFFSET;
        break;

    case STATE_SHOOT_1 + WAITING_OFFSET:
        if (shooter.isDone()) {
            currentState = STATE_PATH_2;  // Continue to next path
        }
        break;
    */
}