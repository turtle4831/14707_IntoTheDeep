package org.firstinspires.ftc.teamcode.Pedrio;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedforward.BasicFeedforward;
import com.ThermalEquilibrium.homeostasis.Filters.Estimators.KalmanEstimator;
import com.ThermalEquilibrium.homeostasis.Parameters.FeedforwardCoefficients;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.ThermalEquilibrium.homeostasis.Systems.PositionVelocitySystem;

import java.util.function.DoubleSupplier;

public class SystemController {
    double Q = 0.3;
    double R = 3;
    int N = 3;
    BasicPID posControl;
    BasicPID veloControl;
    DoubleSupplier positionSupplier;
    DoubleSupplier velocitySupplier;

    KalmanEstimator positionFilter;
    KalmanEstimator velocityFilter;
    FeedforwardCoefficients coefficientsFF;  //new FeedforwardCoefficients(0.1,0.3,0.001);
    BasicFeedforward feedforward;

    PositionVelocitySystem system;


    public SystemController(PIDCoefficients positionPid, PIDCoefficients velocityPid, DoubleSupplier positionSupplier, DoubleSupplier velocitySupplier, double Q, double R, int N,FeedforwardCoefficients ffCoefs  ){
        posControl = new BasicPID(positionPid);
        veloControl = new BasicPID(velocityPid);
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
        positionFilter = new KalmanEstimator(positionSupplier,Q,R,N);
        velocityFilter = new KalmanEstimator(velocitySupplier,Q,R,N);

        feedforward = new BasicFeedforward(ffCoefs);

        system =
                new PositionVelocitySystem(positionFilter,
                        velocityFilter,feedforward,posControl,veloControl);

    }

    public double update(double position, double velocity, double accel){
        return system.update(position,velocity,accel);
    }

}
