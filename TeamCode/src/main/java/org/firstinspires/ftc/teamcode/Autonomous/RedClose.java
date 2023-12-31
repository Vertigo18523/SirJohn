package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.VisionProcessors.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class RedClose extends BaseOpMode {
    public SirJohn robot;
    public boolean place1 = false;
    public RRMecanum drive;
    public TeamPropDetection.ParkingPosition position = TeamPropDetection.ParkingPosition.CENTER;

    public Trajectory forward;
    public Trajectory toCenter;
    public Trajectory centerToPurple;
    public Trajectory rightInitial;
    public Trajectory leftMiddle;
    public Trajectory updateTrajectory;
    public Trajectory centerMiddle;
    public Trajectory rightMiddle;
    public Trajectory middle;
    public Trajectory backDrive;

    @Override
    protected Robot setRobot() {
        this.robot = new SirJohn();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }
    //TODO make proper movement
    @Override
    public void onInit(){
        FtcDashboard.getInstance().startCameraStream(robot.camera.streamSource, 0);
        robot.intake.closeClaw();
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.setIsBlue(false);
        robot.outtake.toMiddle();

        backDrive = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-4,0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
//                .addTemporalMarker(0.25,() -> {
//                    robot.slides.move(0,1);
//                    robot.outtake.flip();
//                })
                .build();


        forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(26,-6),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        centerToPurple = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(30,-20), Math.toRadians(0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(33,-20), Math.toRadians(0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        rightInitial = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(24,-26.5), Math.toRadians(0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        leftMiddle = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(22,7), 0,  RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        centerMiddle = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(10,-15), RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        rightMiddle = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(10,0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        robot.camera.init();
        robot.intake.setAutoPos();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );
    }
    @Override
    public void onStart() throws InterruptedException {
        position = robot.camera.getPosition();
        robot.intake.setAutoPos();
        drive.waitForIdle();

        sleep(1000);


        if(position == TeamPropDetection.ParkingPosition.LEFT) {
            drive.followTrajectoryAsync(forward);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(-127));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectoryAsync(leftMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
            place1 = true;
            drive.waitForIdle();



        }


        if(position == TeamPropDetection.ParkingPosition.CENTER){
            drive.waitForIdle();
            drive.followTrajectoryAsync(centerToPurple);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(-130));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectoryAsync(centerMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
            place1 = true;
            drive.waitForIdle();
            drive.waitForIdle();

        }
        if(position == TeamPropDetection.ParkingPosition.RIGHT){
            drive.waitForIdle();
            drive.followTrajectoryAsync(rightInitial);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(-135));
            drive.waitForIdle();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectoryAsync(rightMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
            place1 = true;
            drive.waitForIdle();

        }

//        robot.intake.setAutoPos();





    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();
        List<AprilTagDetection> currentDetections = robot.camera.aprilTag.getDetections();
        if(place1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    if (detection.id == 4 && position == TeamPropDetection.ParkingPosition.LEFT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3, -detection.ftcPose.x-6), 0,  RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-4, -detection.ftcPose.x-7),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker( 1,() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-7, -detection.ftcPose.x-23),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 5 && position == TeamPropDetection.ParkingPosition.CENTER) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3.5, -detection.ftcPose.x-10),0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6.5, -detection.ftcPose.x-10),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(1, () -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })

                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6, -detection.ftcPose.x-28),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 6 && position == TeamPropDetection.ParkingPosition.RIGHT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3.8, -detection.ftcPose.x-6.5), 0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-4, -detection.ftcPose.x-7),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker( 1,() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-7, -detection.ftcPose.x-23),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }

                    if(updateTrajectory != null) {
                        drive.followTrajectoryAsync(updateTrajectory);
                    }

                }
                else{
                    drive.turn(Math.toRadians(-20));
                }
            }
            place1 = false;
        }

    }
}
