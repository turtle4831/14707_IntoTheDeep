// Copyright (c) Choreo contributors

package org.firstinspires.ftc.teamcode.Choreo;




import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.Arrays;
/** A single robot state in a ChoreoTrajectory. */
public class ChoreoTrajectoryState{
    private static final double FIELD_LENGTH_METERS = 16.5410515;

    /** The timestamp of this state, relative to the beginning of the trajectory. */
    public final double timestamp;

    /** The X position of the state in meters. */
    public final double x;

    /** The Y position of the state in meters. */
    public final double y;

    /** The heading of the state in radians, with 0 being in the +X direction. */
    public final double heading;

    /** The velocity of the state in the X direction in m/s. */
    public final double velocityX;

    /** The velocity of the state in the Y direction in m/s. */
    public final double velocityY;

    /** The angular velocity of the state in rad/s. */
    public final double angularVelocity;

    /**
     * The force on each module in the X direction in Newtons Module forces appear in the following
     * order: [FL, FR, BL, BR]
     */
    public final double[] moduleForcesX;

    /**
     * The force on each module in the Y direction in Newtons Module forces appear in the following
     * order: [FL, FR, BL, BR]
     */
    public final double[] moduleForcesY;

    /**
     * Constructs a ChoreoTrajectoryState with the specified parameters.
     *
     * @param timestamp The timestamp of this state, relative to the beginning of the trajectory.
     * @param x The X position of the state in meters.
     * @param y The Y position of the state in meters.
     * @param heading The heading of the state in radians, with 0 being in the +X direction.
     * @param velocityX The velocity of the state in the X direction in m/s.
     * @param velocityY The velocity of the state in the Y direction in m/s.
     * @param angularVelocity The angular velocity of the state in rad/s.
     * @param moduleForcesX The force on the swerve modules in the X direction in Newtons.
     * @param moduleForcesY The force on the swerve modules in the Y direction in Netwons.
     */
    public ChoreoTrajectoryState(
            double timestamp,
            double x,
            double y,
            double heading,
            double velocityX,
            double velocityY,
            double angularVelocity,
            double[] moduleForcesX,
            double[] moduleForcesY) {
        this.timestamp = timestamp;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.angularVelocity = angularVelocity;
        this.moduleForcesX = moduleForcesX;
        this.moduleForcesY = moduleForcesY;
    }

    /**
     * Returns the pose at this state.
     *
     * @return the pose at this state.
     */
    public Pose2d getPose() {
        return new Pose2d(x, y, new Rotation2d(heading));
    }


    /**
     * Returns this state, mirrored across the field midline.
     *
     * @return this state, mirrored across the field midline.
     */
    public ChoreoTrajectoryState flipped() {
        return new ChoreoTrajectoryState(
                this.timestamp,
                FIELD_LENGTH_METERS - this.x,
                this.y,
                Math.PI - this.heading,
                -this.velocityX,
                this.velocityY,
                -this.angularVelocity,
                Arrays.stream(this.moduleForcesX).map(x -> -x).toArray(),
                this.moduleForcesY);
    }
}