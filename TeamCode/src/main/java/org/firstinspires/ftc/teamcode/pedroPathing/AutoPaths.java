package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

/**
 * CREATE THIS AS A SEPARATE FILE: AutoPaths.java
 *
 * Defines all paths for the autonomous routine
 *
 * Each PathChain represents a sequence of movements.
 * You can create paths using:
 * - BezierLine: Straight line between two points
 * - BezierCurve: Curved path through control points
 *
 * Heading interpolation options:
 * - setLinearHeadingInterpolation: Smoothly rotate from start to end heading
 * - setConstantHeadingInterpolation: Keep heading constant throughout path
 * - setTangentialHeadingInterpolation: Face the direction of movement
 */
public class AutoPaths {

    public final PathChain path1;
    public final PathChain path2;
    public final PathChain path3;
    public final PathChain path4;

    /**
     * Constructs all paths for the autonomous routine
     * @param follower The Follower instance used to build paths
     */
    public AutoPaths(Follower follower) {
        // Path 1: Move from starting position to first waypoint
        // Goes from (22.6, 120) to (48, 96) while facing 180°
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(22.600, 120.000),  // Start position
                        new Pose(48.000, 96.000)     // End position
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),  // Start heading
                        Math.toRadians(180)   // End heading
                )
                .build();

        // Path 2: Move forward slightly
        // Goes from (48, 96) to (48, 84) while maintaining 180° heading
        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(48.000, 96.000),
                        new Pose(48.000, 84.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();

        // Path 3: Strafe left
        // Goes from (48, 84) to (20, 84) while maintaining 180° heading
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(48.000, 84.000),
                        new Pose(20.000, 84.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();

        // Path 4: Return to scoring position
        // Goes from (20, 84) back to (48, 96) while maintaining 180° heading
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(20.000, 84.000),
                        new Pose(48.000, 96.000)
                ))
                .setLinearHeadingInterpolation(
                        Math.toRadians(180),
                        Math.toRadians(180)
                )
                .build();
    }

    /**
     * Example of creating a multi-segment path (PathChain with multiple paths)
     * Uncomment and modify as needed
     */
    /*
    public PathChain createComplexPath(Follower follower) {
        return follower.pathBuilder()
                // First segment
                .addPath(new BezierLine(
                    new Pose(0, 0),
                    new Pose(24, 24)
                ))
                // Second segment continues from end of first
                .addPath(new BezierLine(
                    new Pose(24, 24),
                    new Pose(48, 24)
                ))
                // Third segment with a curve
                .addPath(new BezierCurve(
                    new Pose(48, 24),
                    new Pose(60, 36),
                    new Pose(72, 24)
                ))
                .setLinearHeadingInterpolation(
                    Math.toRadians(0),
                    Math.toRadians(90)
                )
                .build();
    }
    */
}