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
public class NewIntake implements Component {
    public Servo roller;
    public DcMotor arm;

    Telemetry telemetry;
    public double PULSES_PER_REVOLUTION;

    public int FORWARD;

    public double closed;
    public double open;
    public static int forward;
    public int autoPos;
    public static int backward;
    public static int targetPosition = 0;
    public boolean isForward = false;
    public boolean isTeleOp, forcePosition;
    public double error, prevError = 0, time, prevTime = System.nanoTime() * 1e-9d, power;
    public static double kP = 0.004, kD = 0.00001, kG = 0.05;
    public NewIntake(
            String armName,
            String rollerName,
            HardwareMap hardwareMap,
            Telemetry telemetry,
            boolean isTeleOp,
            double init,
            int forward,
            int backward,
            double closed,
            double open,
            int autoPos
    ) {
        this.roller = hardwareMap.get(Servo.class, rollerName);
        this.arm = hardwareMap.get(DcMotor.class, armName);

        this.telemetry = telemetry;
        this.autoPos = autoPos;
        this.closed = closed;
        this.open = open;
        org.firstinspires.ftc.teamcode.Components.Intake.forward = forward;
        org.firstinspires.ftc.teamcode.Components.Intake.backward = backward;
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

    public void rollForward(){
        isForward = true;
        updatePower();
    }

    public void rollBackward(){
        isForward = false;
        updatePower();
    }

    public void toggleRoller(){
        isForward = !isForward;
        updatePower();
    }

    public void updatePower(){
        //update roller to be negative or positive depending on whether isForward is true
    }

    public void setAutoPos(){
        arm.setTargetPosition(autoPos);
        targetPosition = autoPos;
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
                        "rollerIsForward: " + isForward + "\n" +
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