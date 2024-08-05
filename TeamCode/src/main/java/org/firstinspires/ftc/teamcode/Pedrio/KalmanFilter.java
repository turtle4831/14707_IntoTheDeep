package org.firstinspires.ftc.teamcode.Pedrio;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

public class KalmanFilter{
    private com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter xFilter;
    private com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter yFilter;

    public KalmanFilter(com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter x, com.ThermalEquilibrium.homeostasis.Filters.FilterAlgorithms.KalmanFilter y){
        this.xFilter = x;
        this.yFilter = y;
    }

    public Pose2d loop(Pose2d cameraInput){
        return new Pose2d(
                this.xFilter.estimate(cameraInput.getX()),
                this.yFilter.estimate(cameraInput.getY()),
                new Rotation2d(0)
        );
    }


}