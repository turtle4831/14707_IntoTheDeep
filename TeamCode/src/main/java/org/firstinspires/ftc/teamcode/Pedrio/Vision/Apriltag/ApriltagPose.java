package org.firstinspires.ftc.teamcode.Pedrio.Vision.Apriltag;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

public class ApriltagPose {
    public double heading;
    public double x;
    public double y;
    public ApriltagPose(AprilTagPoseFtc ftcPose){
        this.heading = Math.toRadians(-ftcPose.yaw);
        this.x = ftcPose.x * Math.cos(heading) - ftcPose.y * Math.sin(heading);
        this.y = ftcPose.x * Math.sin(heading) + ftcPose.y * Math.cos(heading);
    }

}
