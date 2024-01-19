package org.firstinspires.ftc.teamcode.Components;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Base.BaseOpMode;
import org.firstinspires.ftc.teamcode.Base.Component;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.VisionProcessors.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class Camera implements Component {

    private final Telemetry telemetry;
    private VisionPortal visionPortal;
    private TeamPropDetection visionProcessor;
    public CameraStreamProcessor streamSource;
    public AprilTagProcessor aprilTag;
    public String deviceName;
    public HardwareMap hardwareMap;

    public Camera(String deviceName, BaseOpMode opMode, HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.deviceName = deviceName;
        this.hardwareMap = hardwareMap;

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1430.614, 1430.614 , 639.518, 377.834)
                .build();



        visionProcessor = new TeamPropDetection(opMode, telemetry);
        streamSource = new CameraStreamProcessor();

        android.util.Size cameraResolution = new android.util.Size(1280, 720);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, deviceName))
                .addProcessors(visionProcessor, aprilTag, streamSource)
                .setCameraResolution(cameraResolution)
                .build();

    }

    @Override
    public void init() {

    }

    @Override
    public void start() {

    }

    @Override
    public void update() {}


    @Override
    public String getTelemetry() {
//        telemetryAprilTag();
        return visionProcessor.getTelemetry() + "\n" +
                telemetryAprilTag();
    }

    public void setIsBlue(boolean isBlue){
        visionProcessor.setIsBlue(isBlue);
    }
    public boolean getIsBlue(){
        return visionProcessor.getIsBlue();
    }

    public TeamPropDetection.PropPosition getPosition() {
        return visionProcessor.getPosition();
    }

    public String telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        String out = "";
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                out += String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name) + "\n";
                out += String.format("XYZ %6.1f %6.1f %6.1f  (inch)\n", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z);
                out += String.format("PRY %6.1f %6.1f %6.1f  (deg)\n", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw);
                out += String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)\n", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation);
            } else {
                out += String.format("\n==== (ID %d) Unknown\n", detection.id);
                out += String.format("Center %6.0f %6.0f   (pixels)\n", detection.center.x, detection.center.y);
            }
        }   // end for() loop

        // Add "key" information to telemetry
        out += "\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.\n";
        out += "PRY = Pitch, Roll & Yaw (XYZ Rotation)\n";
        out += "RBE = Range, Bearing & Elevation" + "\n";
        return out;

    }


    // maybe later
//    public void driveToTag(int wantedAprilTag){
//        ArrayList<AprilTagDetection> list = aprilTag.getDetections();
//        for(int i = 0; i < list.size(); i++){
//            if(list.get(i).id == wantedAprilTag){
//
//            }
//        }
//    }

}