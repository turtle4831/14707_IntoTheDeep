package org.firstinspires.ftc.teamcode.Pedrio.PointToPoint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;

public class Point {
    private Pose2d PointPose;
    private CommandBase commands;
    public Point(Pose2d pose, CommandBase commands){
        this.PointPose = pose;
        this.commands = commands;
    }
    public Point(Pose2d pose2d){
        this.PointPose = pose2d;
    }
    public Pose2d getPose(){
        return PointPose;
    }
    public void runCommand(){
        CommandScheduler.getInstance().schedule(commands);
    }
    public CommandBase getCommands(){
        return this.commands;
    }
}
