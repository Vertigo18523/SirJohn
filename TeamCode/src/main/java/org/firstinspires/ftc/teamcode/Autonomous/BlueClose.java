package org.firstinspires.ftc.teamcode.Autonomous;

import org.firstinspires.ftc.teamcode.VisionProcessors.*;

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
public class BlueClose extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
    public TeamPropDetection.ParkingPosition position = TeamPropDetection.ParkingPosition.CENTER;

    public Trajectory forward;
    public Trajectory toCenter;
    public Trajectory updateTrajectory;
    private boolean place1 = false;
    public Trajectory extraForward;
    public Trajectory leftInitial;
    public Trajectory leftMiddle;
    public Trajectory centerMiddle;
    public Trajectory rightMiddle;

    @Override
    protected Robot setRobot() {
        this.robot = new SirJohn();
        return this.robot;
    }

    @Override
    protected boolean setTeleOp() {
        return false;
    }

    @Override
    public void onInit(){
        robot.intake.closeClaw();
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);

        robot.camera.setIsBlue(true);
        robot.outtake.toMiddle();


        forward = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(27,-2),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                .build();
        extraForward = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28,2), Math.toRadians(0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(40,2), Math.toRadians(0),RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        leftInitial = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(28.5,14.5), RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        rightMiddle = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(25,-8),Math.toRadians(0), RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        centerMiddle = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(14,14),Math.toRadians(0), RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        leftMiddle = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(2,11), RRMecanum.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        robot.camera.init();




    }
    @Override
    public void onStart() throws InterruptedException {
        position = robot.camera.getPosition();
        robot.intake.setAutoPos();
        drive.waitForIdle();
        sleep(1000);


        if(position == TeamPropDetection.ParkingPosition.LEFT) {
            drive.waitForIdle();
            drive.followTrajectoryAsync(leftInitial);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(75));
            drive.waitForIdle();
            robot.intake.setAutoPos();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(leftMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
            place1 = true;
            drive.waitForIdle();
        }


        if(position == TeamPropDetection.ParkingPosition.CENTER){
            drive.waitForIdle();
            drive.followTrajectoryAsync(extraForward);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(63));
            drive.waitForIdle();
            robot.intake.setAutoPos();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(centerMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
            place1 = true;
            drive.waitForIdle();
        }
        if(position == TeamPropDetection.ParkingPosition.RIGHT){
            drive.followTrajectoryAsync(forward);
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(75));
            drive.waitForIdle();
            robot.intake.setAutoPos();
            robot.intake.toggleClaw();
            drive.setPoseEstimate(new Pose2d());
            drive.followTrajectory(rightMiddle);
            drive.setPoseEstimate(new Pose2d());
            drive.waitForIdle();
            sleep(500);
//            sleep(1000);
            place1 = true;
            drive.waitForIdle();


        }
        robot.intake.setAutoPos();




    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();
        List<AprilTagDetection> currentDetections = robot.camera.aprilTag.getDetections();
        if(place1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    if (detection.id == 1 && position == TeamPropDetection.ParkingPosition.LEFT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-4, -detection.ftcPose.x-7), 0,  RRMecanum.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(20,() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6, -detection.ftcPose.x-5),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6, -detection.ftcPose.x+15),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })
                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 2 && position == TeamPropDetection.ParkingPosition.CENTER) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-5, -detection.ftcPose.x-6),0, RRMecanum.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(17,() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6.5, -detection.ftcPose.x-5),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6, -detection.ftcPose.x+12),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })
                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 3 && position == TeamPropDetection.ParkingPosition.RIGHT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-5, -detection.ftcPose.x-6.5), 0, RRMecanum.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(18,() -> {
                                    robot.slides.move(500,1);
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6.5, -detection.ftcPose.x-4),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-2, -detection.ftcPose.x+19),0, RRMecanum.getVelocityConstraint(5, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                })
                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    if(updateTrajectory != null) {
                        drive.followTrajectoryAsync(updateTrajectory);
                    }

                }
            }
            place1 = false;
        }

    }
}