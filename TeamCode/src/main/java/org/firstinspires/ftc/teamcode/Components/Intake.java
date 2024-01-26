package org.firstinspires.ftc.teamcode.Components;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Base.Component;
@Config
public class Intake implements Component {
    public Servo claw;
    public DcMotor arm;

    Telemetry telemetry;
    public double PULSES_PER_REVOLUTION;

    public int FORWARD;

    public double closed;
    public double midClawPos;
    public double open;
    public static int forward;
    public int autoPos;
    public static int backward;
    public static int targetPosition = 0;
    public boolean isClosed = false;
    public boolean isTeleOp, forcePosition;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP = 0.004, kD = 0.00001, kG = 0.03;
    public boolean manualOverrideIsOn = true;
    public Intake(
            String armName,
            String clawName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double init,
            int forward,
            int backward,
            double closed,
            double open,
            int autoPos,
            double midClawPos
    ) {
        this.claw = hardwareMap.get(Servo.class, clawName);
        this.arm = hardwareMap.get(DcMotor.class, armName);

        this.telemetry = telemetry;
        this.autoPos = autoPos;
        this.closed = closed;
        this.midClawPos = midClawPos;
        this.open = open;
        Intake.forward = forward;
        Intake.backward = backward;
        this.PULSES_PER_REVOLUTION = 384.5;
        this.isTeleOp = isTeleOp;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }
    @Override
    public void init() {
        arm.setDirection(DcMotorSimple.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm.setTargetPosition(backward);
        targetPosition = backward;


    }

    @Override
    public void start() {

    }

    @Override
    public void update() {
        update(forcePosition);
        getTelemetry();
        telemetry.update();

    }
    public void update(boolean forcePosition){
        error = targetPosition - getCurrentPosition();
        time = System.nanoTime() * 1e-9d;
        this.forcePosition = forcePosition;
//        if (forcePosition || arm.getCurrentPosition() > (2.731 * arm.PULSES_PER_REVOLUTION)) {
        power = (kP * error) + (kD * -(error - prevError) / (time - prevTime)) + (kG * Math.cos(Math.toRadians(targetPosition * (PULSES_PER_REVOLUTION / 360))));
//        }
        if (forcePosition) {
            power = 0;
        }
        if (!isBusy()) {
            this.forcePosition = false;
        }
        setPower(power);
        //setPower(0);
        prevError = error;
        prevTime = time;
    }

    public void closeClaw(){
        isClosed = true;
        updatePos();
    }

    public void openClaw(){
        isClosed = false;
        updatePos();
    }
    public void setAutoPos(){
        arm.setTargetPosition(autoPos);
        targetPosition = autoPos;
    }
    public void clawToMiddle(){
        claw.setPosition(midClawPos);
    }

    public void toggleClaw(){
        isClosed = !isClosed;
        updatePos();
    }

    public void updatePos(){
        claw.setPosition(isClosed ? closed : open);
    }

    public void setArmPos(int pos){
        arm.setTargetPosition(pos);
        targetPosition = pos;
    }

    @Override
    public String getTelemetry() {
        return
            "CurrentPosition: " + getCurrentPosition() + "\n" +
            "TargetPosition: " + targetPosition + "\n" +
            "arm.getTargetPosition: " + arm.getTargetPosition() + "\n" +
            "error: " + error + "\n" +
            "power: " + arm.getPower() + "\n" +
            "clawIsClosed: " + isClosed + "\n" +
            "Arm position: " + arm.getCurrentPosition() + "\n" +
            "Arm zero power behavior: " + arm.getZeroPowerBehavior();
    }

    public void toggleArm(){
        if(targetPosition == forward){
            arm.setTargetPosition(backward);
            targetPosition = backward;
        }
        else{
            arm.setTargetPosition(forward);
            targetPosition = forward;
        }
    }




    public void toBackwardForce() {
        update(true);
    }

    //TODO figure out what 2.731 means

    public boolean isBusy() {
        return Math.abs(error) > 10;
    }
    public void setPower(double motorPower) {
        if (motorPower > 1) motorPower = 1;
        arm.setPower(motorPower);
    }
    public int getCurrentPosition() {
        return arm.getCurrentPosition();
    }
}
