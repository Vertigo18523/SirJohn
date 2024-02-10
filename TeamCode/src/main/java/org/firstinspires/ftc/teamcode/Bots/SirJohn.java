package org.firstinspires.ftc.teamcode.Bots;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Robot;
import org.firstinspires.ftc.teamcode.Components.Camera;
import org.firstinspires.ftc.teamcode.Components.Crossbow;
import org.firstinspires.ftc.teamcode.Components.Hanger;
import org.firstinspires.ftc.teamcode.Components.Intake;
import org.firstinspires.ftc.teamcode.Components.Mecanum;
import org.firstinspires.ftc.teamcode.Components.Outtake;
import org.firstinspires.ftc.teamcode.Components.Slides;

public class SirJohn extends Robot {

    public Camera camera;
    public boolean isTeleOp;
    public Intake intake;
    public Outtake outtake;
    public Mecanum mecanum;
    public Crossbow crossbow;
    public Hanger hanger;
    public Slides slides;
    public IMU imu;
    public DistanceSensor grabberSense;

    @Override
    protected void mapHardware(HardwareMap hardwareMap, Telemetry telemetry, BaseOpMode opMode, boolean isTeleOp) {
        this.isTeleOp = isTeleOp;
        this.grabberSense = hardwareMap.get(DistanceSensor.class, "grabberSense");
        this.intake = new Intake("arm", "claw", hardwareMap, telemetry, isTeleOp, 0, -4,155,0.7,0.1,-120, 0.2);
        this.outtake = new Outtake("spin", hardwareMap, telemetry, 0.035, 0.84,0.08, 0.675);
        this.intake = new Intake("arm", "claw", hardwareMap, telemetry, isTeleOp, 3, 0,160,0.95,0.1,-60, 0.2);
        this.outtake = new Outtake("spin", hardwareMap, telemetry, 0.032, 0.84,0.08, 0.675);
        this.hanger = new Hanger("hanger", hardwareMap, telemetry, isTeleOp, 0,0,0);
        this.crossbow = new Crossbow("crossbow", hardwareMap, telemetry, 0.7, 1);
        this.slides = new Slides("rightArm","leftArm" , hardwareMap, telemetry, isTeleOp, 0, 2240, 0,90,800, 0.2);
        if(isTeleOp) {
            this.mecanum = new Mecanum(hardwareMap, "frontLeft", "frontRight", "backLeft", "backRight", telemetry);
            this.mecanum.fl.setDirection(DcMotorSimple.Direction.FORWARD);
            this.mecanum.fr.setDirection(DcMotorSimple.Direction.REVERSE);
            this.mecanum.bl.setDirection(DcMotorSimple.Direction.FORWARD);
            this.mecanum.br.setDirection(DcMotorSimple.Direction.REVERSE);
            addComponents(mecanum);
        }
        this.camera = new Camera("Webcam 1", opMode, hardwareMap, telemetry);
        this.imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        addComponents(camera, intake, outtake, crossbow, slides, hanger);
    }
}
