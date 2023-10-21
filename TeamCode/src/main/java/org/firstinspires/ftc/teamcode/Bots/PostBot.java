package org.firstinspires.ftc.teamcode.Bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.ArmRotation;
import org.firstinspires.ftc.teamcode.Components.Grabber;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.Components.Odometry;
import org.firstinspires.ftc.teamcode.Components.PIDMecanum;
import org.firstinspires.ftc.teamcode.Components.Slides;

public class PostBot extends Robot {
    public boolean isTeleOp;
    public OldCamera camera;
    public ArmRotation rotation;
    public Slides arm;
    public Grabber grabber;
    public Mecanum mecanum;
    public PIDMecanum pidMecanum;
    public Odometry odo;

    @Override
    public void mapHardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, boolean isTeleOp) {
        this.isTeleOp = isTeleOp;
        this.camera = new OldCamera(opMode, "Webcam 1", hardwareMap, telemetry);
        this.rotation = new ArmRotation(
                "rotation",
                hardwareMap,
                telemetry,
                isTeleOp,
                0,
                0.302,
                0
        );
        this.arm = new Slides(
                "arm",
                "leftArm",
                hardwareMap,
                telemetry,
                isTeleOp,
                this.rotation,
                0.754,
                0.884,
                1.183,
                1.769,
                2.835,
                1.769,
                3.901,
                5.904,
                7.802
        );
        rotation.addSlides(arm);
        this.grabber = new Grabber(
                opMode,
                "grabber",
                hardwareMap,
                telemetry,
                0.7,
                0.35
        );
        if (isTeleOp) {
            this.mecanum = new Mecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", telemetry);
            this.mecanum.fl.setDirection(DcMotorSimple.Direction.REVERSE);
            this.mecanum.fr.setDirection(DcMotorSimple.Direction.FORWARD);
            this.mecanum.bl.setDirection(DcMotorSimple.Direction.REVERSE);
            this.mecanum.br.setDirection(DcMotorSimple.Direction.FORWARD);
            addComponents(mecanum);
        } else {
            this.odo = new Odometry(hardwareMap, "backRight", "frontLeft", "frontRight");
            odo.leftDir = Odometry.EncoderDirection.REVERSE;
            odo.strafeDir = Odometry.EncoderDirection.REVERSE;
            odo.rightDir = Odometry.EncoderDirection.REVERSE;
            this.pidMecanum = new PIDMecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", odo);
            this.pidMecanum.positionTolerance = 0.5;
            this.pidMecanum.fl.setDirection(DcMotorSimple.Direction.REVERSE);
            this.pidMecanum.fr.setDirection(DcMotorSimple.Direction.FORWARD);
            this.pidMecanum.bl.setDirection(DcMotorSimple.Direction.REVERSE);
            this.pidMecanum.br.setDirection(DcMotorSimple.Direction.FORWARD);
            addComponents(pidMecanum);
       }

        addComponents(camera, grabber, rotation, arm);
    }
}
