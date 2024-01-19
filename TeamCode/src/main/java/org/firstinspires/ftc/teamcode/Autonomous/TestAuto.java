package org.firstinspires.ftc.teamcode.Autonomous;

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
import org.firstinspires.ftc.teamcode.VisionProcessors.TeamPropDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Config
public class TestAuto extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
    public TeamPropDetection.PropPosition position = TeamPropDetection.PropPosition.CENTER;
    public boolean place1 = false;

    public Trajectory tester;
    public Trajectory updateTester;
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
        FtcDashboard.getInstance().startCameraStream(robot.camera.streamSource, 0);
        drive = new RRMecanum(hardwareMap);
        Pose2d startPose = new Pose2d();
        drive.setPoseEstimate(startPose);
        robot.camera.setIsBlue(false);
        robot.camera.init();
        robot.outtake.toMiddle();
        robot.intake.toggleArm();

        tester = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(1,0), RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();
        robot.camera.init();
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

    }
    @Override
    public void onStart() throws InterruptedException{
        position = TeamPropDetection.PropPosition.CENTER;
        robot.outtake.toMiddle();
        drive.waitForIdle();
        drive.followTrajectoryAsync(tester);
        drive.waitForIdle();
        place1 = true;


    }
    @Override
    public void onUpdate(){
        drive.update();
        robot.camera.update();
        robot.camera.telemetryAprilTag();
        telemetry.update();

        List<AprilTagDetection> currentDetections = robot.camera.aprilTag.getDetections();
        if(place1) {
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {

                    if (detection.id == 4 && position == TeamPropDetection.PropPosition.LEFT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTester = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(0,-6), RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                    }
                    else if (detection.id == 5 && position == TeamPropDetection.PropPosition.CENTER) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTester = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(detection.ftcPose.y-2,0), RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                    }
                    else if (detection.id == 6 && position == TeamPropDetection.PropPosition.RIGHT) {
                        drive.setPoseEstimate(new Pose2d());
                        updateTester = drive.trajectoryBuilder(new Pose2d())
                                .lineTo(new Vector2d(10,-6), RRMecanum.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                                        RRMecanum.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                                .build();

                    }
                    if(updateTester != null) {
                        drive.followTrajectoryAsync(updateTester);
                    }

                }
            }
            place1 = false;
        }
    }
}
