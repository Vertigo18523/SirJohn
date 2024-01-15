package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Bots.SirJohn;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.RRMecanum;

@Autonomous
public class RedClosePark extends BaseOpMode {
    public SirJohn robot;
    public RRMecanum drive;
    public Trajectory offWall, toPark;

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
        drive = new RRMecanum(hardwareMap);
        offWall = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(4)
                .build();
        toPark = drive.trajectoryBuilder(new Pose2d())
                .forward(38)
                .build();
        robot.outtake.toMiddle();
    }
    @Override
    public void onStart() throws InterruptedException {
//        drive.followTrajectory(offWall);
//        drive.waitForIdle();
//        drive.setPoseEstimate(new Pose2d());
        drive.followTrajectoryAsync(toPark);
        drive.waitForIdle();
//        drive.setPoseEstimate(new Pose2d());
        robot.outtake.unFlip();
    }
    @Override
    public void onUpdate(){
        drive.update();
    }
}