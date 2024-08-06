package org.firstinspires.ftc.teamcode.Pedrio.PointToPoint;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.geometry.Pose2d;

public class Point {
    private Pose2d PointPose;
    private long wait;
    private CommandBase commands;
    public Point(Pose2d pose, CommandBase commands, long wait){
        this.PointPose = pose;
        this.commands = commands;
        this.wait = wait;
    }
    public Point(Pose2d pose2d, long wait){
        this.PointPose = pose2d;
        this.wait = wait;
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
    public long getWait(){
        return this.wait;
    }
}
