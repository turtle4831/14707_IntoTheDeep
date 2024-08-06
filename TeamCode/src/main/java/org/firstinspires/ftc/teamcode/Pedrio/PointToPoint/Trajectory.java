package org.firstinspires.ftc.teamcode.Pedrio.PointToPoint;

import static java.lang.Thread.sleep;

import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.Pedrio.Config;
import org.firstinspires.ftc.teamcode.Pedrio.SystemController;
import org.firstinspires.ftc.teamcode.Robot.Subsystems.Drivetrain;

import java.util.List;
import java.util.function.DoubleSupplier;

public class Trajectory {
    private Drivetrain drivetrain = new Drivetrain();

    private final SystemController xController = new SystemController(
            new PIDCoefficients(0.001, 0, 0),
            new PIDCoefficients(0.001, 0, 0),
            new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return drivetrain.getFusedPose().getX();
                }
            }
            , new DoubleSupplier() {
                 @Override
                    public double getAsDouble() {
                return drivetrain.getVelocity().x;
                }
            }
            ,
            0.3,
            3,
            3,
            new FeedforwardCoefficients(0.1, 0.3, 0.001)

    ) ;
    private final SystemController yController = new SystemController(
            new PIDCoefficients(0.001, 0, 0),
            new PIDCoefficients(0.001, 0, 0),
            new DoubleSupplier() {
                @Override
                public double getAsDouble() {
                    return drivetrain.getFusedPose().getY();
                }
            }
            , new DoubleSupplier() {
        @Override
        public double getAsDouble() {
            return drivetrain.getVelocity().y;
        }
    }
            ,
            0.3,
            3,
            3,
            new FeedforwardCoefficients(0.1, 0.3, 0.001)

    ) ;

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
    public void follow() throws InterruptedException {
        double x;
        double y;
        double turn;
        for(int i = 0; i <= this.trajectory.size(); i++){
            while(true) {
                x = xController.update(trajectory.get(i).getPose().getX(), 2000,1000);
                y = yController.update(trajectory.get(i).getPose().getY(), 2000,1000);
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
