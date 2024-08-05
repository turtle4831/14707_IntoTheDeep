package org.firstinspires.ftc.teamcode.Pedrio;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import java.util.ArrayList;

public class Config {
    //point to point
    public static double toleranceMin = 0.05;
    public static double toleranceMax = 0.05;

    //camera localization config
    public static ArrayList<Pose2d> apriltagLocations = new ArrayList<>();
    public static Pose2d cameraLocation = new Pose2d(0,0,new Rotation2d(0));
    public void initApriltags(){
        apriltagLocations.add(new Pose2d(0,0,new Rotation2d(0)));//empty because its index 0 == fidiciul 0

    }

}
