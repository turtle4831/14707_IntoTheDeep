package org.firstinspires.ftc.teamcode.Pedrio.Vision.Apriltag;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

import org.firstinspires.ftc.teamcode.Pedrio.Config;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

public class CameraLocalization {

    Pose2d initialCameraLocation = Config.cameraLocation;
    Drivetrain odo = new Drivetrain();
    public Pose2d getPoseFromTag(int tag_id, ApriltagPose pose){
        double x;
        double y;

        Pose2d currentCameraPose = initialCameraLocation.transformBy(new Transform2d(new Translation2d(odo.getPose().x,odo.getPose().y),new Rotation2d(0)));
        Pose2d tag = Config.apriltagLocations.get(tag_id);
        x = currentCameraPose.getX() * Math.cos(pose.heading) - currentCameraPose.getY() * Math.sin(pose.heading);
        y = currentCameraPose.getY() * Math.sin(pose.heading) + currentCameraPose.getY() * Math.cos(pose.heading);

        return new Pose2d(x,y,new Rotation2d(0)).plus(new Transform2d(new Translation2d(tag.getX(),tag.getY()),new Rotation2d(tag.getHeading())));

    }
}
