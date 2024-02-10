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
    public TeamPropDetection.PropPosition position = TeamPropDetection.PropPosition.CENTER;

    public Trajectory forward;
    public Trajectory toCenter;
    public Trajectory updateTrajectory;
    private boolean place1 = false;
    public Trajectory extraForward;
    public Trajectory leftInitial;
    public Trajectory leftMiddle;
    public Trajectory strafeRight;
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
                .lineTo(new Vector2d(28,-4),RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> {
                    robot.intake.setAutoPos();
                })

                .build();
        extraForward = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(28,10), Math.toRadians(0),RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .splineTo(new Vector2d(40,10), Math.toRadians(0),RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> {
                    robot.intake.setAutoPos();
                })
                .build();
        leftInitial = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(30,19), RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .addDisplacementMarker( () -> {
                    robot.intake.setAutoPos();
                })
                .build();
        strafeRight = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(1,-5), RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        rightMiddle = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(20,-2),Math.toRadians(0), RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        centerMiddle = drive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d(7,11),Math.toRadians(0), RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        leftMiddle = drive.trajectoryBuilder(new Pose2d())
                .lineTo(new Vector2d(6,6), RRMecanum.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH), RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();


        robot.camera.init();




    }
    @Override
    public void onStart() throws InterruptedException {
        position = robot.camera.getPosition();
        robot.intake.setAutoPos();
        drive.waitForIdle();
        sleep(1000);


        if(position == TeamPropDetection.PropPosition.LEFT) {
            drive.waitForIdle();
            drive.followTrajectoryAsync(leftInitial);
            robot.intake.setAutoPos();
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(92));
            drive.waitForIdle();
            drive.setPoseEstimate(new Pose2d());
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


        if(position == TeamPropDetection.PropPosition.CENTER){
            drive.waitForIdle();
            drive.followTrajectoryAsync(extraForward);
            robot.intake.setAutoPos();
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(89));
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
        if(position == TeamPropDetection.PropPosition.RIGHT){
            drive.followTrajectoryAsync(forward);
            robot.intake.setAutoPos();
            drive.waitForIdle();
            drive.turnAsync(Math.toRadians(89));
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

                    if (detection.id == 1 && position == TeamPropDetection.PropPosition.LEFT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3.5, -detection.ftcPose.x), 0,  RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(16,() -> {
                                    robot.slides.move(760,1);
                                    robot.intake.claw.close();
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-5.5, -detection.ftcPose.x),0, RRMecanum.getVelocityConstraint(1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                    robot.intake.setAutoPos();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6.5, -detection.ftcPose.x-34),0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 2 && position == TeamPropDetection.PropPosition.CENTER) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3.5, -detection.ftcPose.x-2.3),0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(17,() -> {
                                    robot.slides.move(760,1);
                                    robot.intake.claw.close();
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-5.5, -detection.ftcPose.x-2.3),0, RRMecanum.getVelocityConstraint(1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                    robot.intake.setAutoPos();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-6.5, -detection.ftcPose.x-25),0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

                                .build();
                        drive.turn(Math.toRadians(-detection.ftcPose.roll));

                    }
                    else if (detection.id == 3 && position == TeamPropDetection.PropPosition.RIGHT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTrajectory = drive.trajectoryBuilder(new Pose2d())
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-3.5, -detection.ftcPose.x-3.5), 0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(18,() -> {
                                    robot.slides.move(760,1);
                                    robot.intake.claw.close();
                                    robot.outtake.unFlip();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-5.5, -detection.ftcPose.x-3.5),0, RRMecanum.getVelocityConstraint(1, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .addDisplacementMarker(() -> {
                                    robot.slides.waitForIdle();
                                    robot.slides.move(20,1);
                                    robot.outtake.flip();
                                    robot.intake.setAutoPos();
                                })
                                .splineToConstantHeading(new Vector2d(detection.ftcPose.y-7, -detection.ftcPose.x-21),0, RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))

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