package org.firstinspires.ftc.teamcode.Pedrio.DataFusing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;


public class RegularFusing {


    private double otos_x_weight = 1;
    private double otos_y_weight = 1;
    private double otos_z_weight = 1;

    private double apriltag_x_weight = 1;
    private double apriltag_y_weight = 1;

    private double gyro_z_weight = 1;



    public RegularFusing(){}



    public Pose2d fuse(Pose2d OTOS, Pose2d AprilTagPose, Rotation2d gyro){
        return new Pose2d(
                new Translation2d(
                        (OTOS.getX() * otos_x_weight) + (AprilTagPose.getX() * apriltag_x_weight) /2,
                        (OTOS.getY() * otos_y_weight) + (AprilTagPose.getY() * apriltag_y_weight) /2

                ),
                new Rotation2d(
                        (OTOS.getHeading() * otos_z_weight) + (gyro.getDegrees() * gyro_z_weight) / 2
                )
        );
    }
}
