package org.firstinspires.ftc.teamcode.Pedrio.PointToPoint;

import static java.lang.Thread.sleep;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Pedrio.Config;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

import java.util.List;

public class Trajectory {
    private Drivetrain drivetrain = new Drivetrain();
    private final PIDController xController = new PIDController(0.001,0,0);
    private final PIDController yController = new PIDController(0.001,0,0);
    private final PIDController turnController = new PIDController(0.001,0,0);

    private List<Point> trajectory;

    public Trajectory(List<Point> trajectory){
        this.trajectory = trajectory;
        this.drivetrain.configureOtos(new SparkFunOTOS.Pose2D(
                this.trajectory.get(0).getPose().getX(),
                this.trajectory.get(0).getPose().getY(),
                this.trajectory.get(0).getPose().getHeading()
        ));
    }

    public Point getPoint(int currentIndex){
        return this.trajectory.get(currentIndex);
    }



    public void follow() throws InterruptedException {
        double x;
        double y;
        double turn;
        for(int i = 0; i <= this.trajectory.size(); i++){
            while(true) {
                x = xController.calculate(drivetrain.getPose().x, trajectory.get(i).getPose().getX());
                y = yController.calculate(drivetrain.getPose().y, trajectory.get(i).getPose().getY());
                turn = turnController.calculate(drivetrain.getPose().h, trajectory.get(i).getPose().getHeading());

                drivetrain.driveFieldCentric(x, y, turn, drivetrain.getRawIMUHeadingDegrees());

                if (tolerance(averageError(x, y, turn), Config.toleranceMin, Config.toleranceMax)) {
                    if (this.trajectory.get(i).getCommands() != null) {
                        this.trajectory.get(i).runCommand();
                    }
                    if(this.trajectory.get(i).getWait() != 0 ){
                        sleep(this.trajectory.get(i).getWait()); //change this to make a call to the opmode
                    }
                    break;
                }

            }
        }

    }
    private boolean tolerance(double value,double min,double max){
        return value >= min && value <= max;
    }
    private double averageError(double x, double y, double turn){
        return (x + y + turn) / 3;
    }

}
