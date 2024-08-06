package org.firstinspires.ftc.teamcode.Robot.Subsystems;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Pedrio.DataFusing.RegularFusing;
import org.firstinspires.ftc.teamcode.Pedrio.PedrioSubsystem;
import org.firstinspires.ftc.teamcode.Pedrio.Vision.Apriltag.ApriltagPose;
import org.firstinspires.ftc.teamcode.Pedrio.Vision.Apriltag.CameraLocalization;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;


public class Drivetrain extends PedrioSubsystem {

    private final Hardware robot = Hardware.getInstance();
    public MecanumDrive drive = new MecanumDrive(robot.FlMotor, robot.FrMotor, robot.BlMotor, robot.BrMotor);
    public final SparkFunOTOS myOtos = robot.otos;
    private CameraLocalization cameraLocalization = new CameraLocalization();
    private RegularFusing dataFuser = new RegularFusing();
    private double LENGTH = 14;
    private double WIDTH = 14;

    private final MecanumDriveKinematics kine = new MecanumDriveKinematics(
            new Translation2d(LENGTH / 2, WIDTH / 2),
            new Translation2d(LENGTH / 2, -WIDTH / 2),
            new Translation2d(-LENGTH / 2, WIDTH / 2),
            new Translation2d(-LENGTH / 2, -WIDTH / 2)
    );


    public Pose2d getFusedPose(){

        Pose2d apriltagpose;
        Pose2d cameraPose;
        ArrayList<Pose2d> aprilTagPoses = new ArrayList<>();
        int i = 0;
        for(AprilTagDetection detection : robot.aprilTag.getDetections()){
            if(detection.metadata != null){
                //create a list and add all detections to it then divide it by size of list
                ApriltagPose tagpose = new ApriltagPose(detection.ftcPose);
                aprilTagPoses.add(cameraLocalization.getPoseFromTag(detection.id, tagpose));
                i++;
            }
        }
        double x = 0;
        double y = 0;
        if(aprilTagPoses.isEmpty()){
            cameraPose = null;
        }else {
            for (Pose2d pose : aprilTagPoses) {
                x += pose.getX();
                y += pose.getY();
            }
            cameraPose = new Pose2d(x / i, y / i, new Rotation2d(0));
            aprilTagPoses.clear();
        }
        Pose2d otosPose = new Pose2d(
                myOtos.getPosition().x,
                myOtos.getPosition().y,
                new Rotation2d(myOtos.getPosition().h)
        );
        if(cameraPose != null) {
            return dataFuser.fuse(otosPose, cameraPose, new Rotation2d(getRawIMUHeadingDegrees()));
        }
        return otosPose;
    }

    public SparkFunOTOS.Pose2D getVelocity(){
        return myOtos.getVelocity();
    }

    public void driveFieldCentric(double x, double y, double turn, double gyroAngle) {
        drive.driveFieldCentric(x, y, turn, gyroAngle);
    }
    public void driveFieldCentricWithWheelSpeeds(ChassisSpeeds speeds){
        drive.driveFieldCentric(speeds.vxMetersPerSecond,speeds.vyMetersPerSecond,speeds.omegaRadiansPerSecond,getRawIMUHeadingDegrees());
    }

    public SparkFunOTOS.Pose2D getPose() {
        return myOtos.getPosition();
    }

    public double getRawIMUHeadingDegrees() {
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getRawIMUHeadingRadians() {
        return robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }


    @Override
    public void init() {

    }

    @Override
    public void periodic() {

    }

    public void configureOtos(SparkFunOTOS.Pose2D startingPose) {


        // Set the desired units for linear and angular measurements. Can be either
        // meters or inches for linear, and radians or degrees for angular. If not
        // set, the default is inches and degrees. Note that this setting is not
        // persisted in the sensor, so you need to set at the start of all your
        // OpModes if using the non-default value.
        // myOtos.setLinearUnit(DistanceUnit.METER);
        myOtos.setLinearUnit(DistanceUnit.INCH);
        // myOtos.setAngularUnit(AnguleUnit.RADIANS);
        myOtos.setAngularUnit(AngleUnit.DEGREES);

        // Assuming you've mounted your sensor to a robot and it's not centered,
        // you can specify the offset for the sensor relative to the center of the
        // robot. The units default to inches and degrees, but if you want to use
        // different units, specify them before setting the offset! Note that as of
        // firmware version 1.0, these values will be lost after a power cycle, so
        // you will need to set them each time you power up the sensor. For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        myOtos.setOffset(offset);

        // Here we can set the linear and angular scalars, which can compensate for
        // scaling issues with the sensor measurements. Note that as of firmware
        // version 1.0, these values will be lost after a power cycle, so you will
        // need to set them each time you power up the sensor. They can be any value
        // from 0.872 to 1.127 in increments of 0.001 (0.1%). It is recommended to
        // first set both scalars to 1.0, then calibrate the angular scalar, then
        // the linear scalar. To calibrate the angular scalar, spin the robot by
        // multiple rotations (eg. 10) to get a precise error, then set the scalar
        // to the inverse of the error. Remember that the angle wraps from -180 to
        // 180 degrees, so for example, if after 10 rotations counterclockwise
        // (positive rotation), the sensor reports -15 degrees, the required scalar
        // would be 3600/3585 = 1.004. To calibrate the linear scalar, move the
        // robot a known distance and measure the error; do this multiple times at
        // multiple speeds to get an average, then set the linear scalar to the
        // inverse of the error. For example, if you move the robot 100 inches and
        // the sensor reports 103 inches, set the linear scalar to 100/103 = 0.971
        myOtos.setLinearScalar(1.0);
        myOtos.setAngularScalar(1.0);

        // The IMU on the OTOS includes a gyroscope and accelerometer, which could
        // have an offset. Note that as of firmware version 1.0, the calibration
        // will be lost after a power cycle; the OTOS performs a quick calibration
        // when it powers up, but it is recommended to perform a more thorough
        // calibration at the start of all your OpModes. Note that the sensor must
        // be completely stationary and flat during calibration! When calling
        // calibrateImu(), you can specify the number of samples to take and whether
        // to wait until the calibration is complete. If no parameters are provided,
        // it will take 255 samples and wait until done; each sample takes about
        // 2.4ms, so about 612ms total
        myOtos.calibrateImu();

        // Reset the tracking algorithm - this resets the position to the origin,
        // but can also be used to recover from some rare tracking errors
        myOtos.resetTracking();

        // After resetting the tracking, the OTOS will report that the robot is at
        // the origin. If your robot does not start at the origin, or you have
        // another source of location information (eg. vision odometry), you can set
        // the OTOS location to match and it will continue to track from there.

        myOtos.setPosition(startingPose);

        // Get the hardware and firmware version
        SparkFunOTOS.Version hwVersion = new SparkFunOTOS.Version();
        SparkFunOTOS.Version fwVersion = new SparkFunOTOS.Version();
        myOtos.getVersionInfo(hwVersion, fwVersion);
    }
}
