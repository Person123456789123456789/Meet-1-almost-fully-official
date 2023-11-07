package org.firstinspires.ftc.teamcode.drive.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.armsNStuff;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "RedRight", group = "RED")
public class RedRight extends LinearOpMode {
    armsNStuff arm;

    OpenCvCamera webcam;
    GamePropRight.gamePropPosition propPosition = GamePropRight.gamePropPosition.LEFT;
    SampleMecanumDrive robot;
    @Override
    public void runOpMode() throws InterruptedException {

        arm=new armsNStuff(hardwareMap);

        telemetry.addData("Start OpMode", "BLUE LEFT");
        telemetry.update();
        startCamera();
        telemetry.addData("Selected Starting Position", propPosition);
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        while (!isStopRequested() && !opModeIsActive())
        {propPosition = getPropPosition();
            telemetry.addData("Identified Prop Location", propPosition);
            telemetry.update();
        }

        if (opModeIsActive() && !isStopRequested()) {
            //Build parking trajectory based on last detected target by vision
            webcam.stopStreaming(); //Stop Webcam to preserve Controlhub cycles.
            runAutonoumousMode();
        }
    }

    public void runAutonoumousMode() {

        Pose2d initPose = new Pose2d(0, 0, Math.toRadians(0.0)); // Starting Pose --Update CoOrdinates
        Pose2d yellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d purplePixelPose = new Pose2d(0, 0, Math.toRadians(0.0));;
        switch (propPosition) { //UPDATE THESE POSITIONS
            case LEFT:
                telemetry.addData("Left Position", "Left");
                yellowPixelPose = new Pose2d(28,-35 , Math.toRadians(60));
                purplePixelPose = new Pose2d(30, 10, Math.toRadians(60.0));
                break;
            case CENTER:
                telemetry.addData("Center Position", "Center");
               yellowPixelPose = new Pose2d(20, -35,Math.toRadians(60) );
                purplePixelPose = new Pose2d(26, 5, Math.toRadians(0.0));
                break;
            case RIGHT:
                yellowPixelPose = new Pose2d(24, -35, Math.toRadians(60));
                telemetry.addData("Right Position", "Right");
                purplePixelPose = new Pose2d(23,-10, Math.toRadians(0));
                break;

        }
        //Pose2d parkingPose = new Pose2d(-25, 75, -90); //UPDATE

        telemetry.update();
        sleep(2000);

        robot = new SampleMecanumDrive(hardwareMap);
        robot.followTrajectorySequence(robot.trajectorySequenceBuilder(initPose) //Starting Pose
                .lineToLinearHeading(purplePixelPose)//Drop Purple Pixel
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {arm.spikeMark(0.3);})
                .waitSeconds(2.0)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {arm.stopPower();})
                .back(10)
                .lineToLinearHeading(yellowPixelPose)
                .waitSeconds(1)
                .back(14)
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {arm.backdrop(1);})
                .waitSeconds(2)
                .back(2)
                //.lineToLinearHeading(parkingPose) //Parking in the back
                .build());

    }
    public void startCamera() {
        //Initialize Camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        webcam.setPipeline(new GamePropRight());
    }
	
	public GamePropRight.gamePropPosition getPropPosition() {
		return GamePropRight.position;
	}
	
}