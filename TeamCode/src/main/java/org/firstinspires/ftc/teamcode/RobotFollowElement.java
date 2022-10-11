package org.firstinspires.ftc.teamcode;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name = "Concept: ERT RobotFollowElement", group = "Concept")

public class RobotFollowElement extends LinearOpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx rearRight;
    private DcMotorEx rearLeft;
    BNO055IMU imu;
    Orientation angles;

    private WebcamName webcamName;
    OpenCvWebcam webcam;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    void assignDrivetrainPower(double xG, double yG, double rxG){
        double rx = (-rxG / 2) * (gamepad1.left_trigger + 1);
        double trig = gamepad1.right_trigger;
        double x = -(((xG * 1.1) / 2) * (1 + trig));
        double y = (yG / 2) * (1 + trig);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);

        rearRight.setPower((y + x - rx) / denominator);
        rearLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        frontLeft.setPower((y + x + rx) / denominator);
    }

    @Override
    public void runOpMode() {
//        webcamName = hardwareMap.get(WebcamName.class, "webcam");

//        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
//        vuforiaParams.vuforiaLicenseKey = "AdOxIVH/////AAABmQ3Y3+Lcd0Moq/LIq3cP7R0pnbrT6a5B42r/wiy//DDZ/bCBYXJ8GnCAMg3z173lTes2aL1pkGIVrocm53OJquzayMah1r7mZtDlbVY5bbEgrSyHiQTXNPqDM7ISanT6Zmkx4qXj9xh3ksg+aR5zIP+6lAp4FXqDbc4/QpIwLQgUw8cy8wfnM/MBIM+Q/2MSmCAemIltguGX50JiPgStcBXq9Yuz+cwHJdol0bj9wmog0Rq3Fm13sUQZF1/J22oUERbKVKlXJ0gjO8Sq28+6K+5Czp5jlmH/d6U+TloIqgf+9RipQlyxzwjfV9INTVR6/0lzcygC3SbvK/ZPcZYEDF+59H2NXUlYL6QQ92NvZ9tM";
//        vuforiaParams.cameraName = webcamName;
//        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
//        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);
//        FtcDashboard.getInstance().startCameraStream(vuforia, 30);

        /**************************** Setup and Initialize IMU (gyro) *****************************/
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        /************************* Setup and Initialize Motors and Servos *************************/
        // Drivetrain Motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        rearRight = hardwareMap.get(DcMotorEx.class, "rearRight");
        rearLeft = hardwareMap.get(DcMotorEx.class, "rearLeft");

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.REVERSE);

        /************************** Initialize FTC Dashboard ************************/
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        /**************************** Initialize Camera ****************************/

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "webcam"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 30);

        RectangleTracking pipeline = new RectangleTracking();
        webcam.setPipeline(pipeline);

        webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPSIDE_DOWN);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        double[] gamepadDirection = {gamepad1.left_stick_x, gamepad1.left_stick_y};
        while(opModeIsActive()) {
            assignDrivetrainPower(gamepad1.left_stick_x, gamepad1.left_stick_y, pipeline.getElementsAnalogCoordinates()[0]);
        }
    }

    class RectangleTracking extends OpenCvPipeline
    {
        Mat imgHSV = new Mat();
        Mat thresholdMat0 = new Mat();
        Mat thresholdMat1 = new Mat();
        Mat thresholdMat = new Mat();
        Mat HSLchan0Mat = new Mat();
        Mat HSLchan1Mat = new Mat();
        Mat thresholdMatS = new Mat();
        Mat thresholdMatH = new Mat();
        Mat contoursOnFrameMat = new Mat();
        List<MatOfPoint> contoursList = new ArrayList<>();
        List<MatOfPoint> bigContours = new ArrayList<>();
        MatOfPoint bigContour = new MatOfPoint();
        int numContoursFound;

        Size viewportSize;

        double hHSVAverage = 0;
        double sHSVAverage = 0;
        double vHSVAverage = 0;

        int rangeAccuracyH = 10;
        int rangeAccuracyS = 30;

        double[] rectangleCoordinates = {0, 0};

        @Override
        public Mat processFrame(Mat input)
        {
//            telemetry.addData(">", input.size().width);
//            telemetry.update();
//
//            dashboardTelemetry.addData(">", input);
//            dashboardTelemetry.update();

//            return  input;
            //Add a Gausian Blur to make more smooth the contours
            Imgproc.GaussianBlur(input, input, new Size(5, 5), 20);

            //Convert image from RGB to HSV
            Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_RGB2HSV);

            //Assing viewportSize variable to viewport size in pixels
            viewportSize = input.size();

            //Getting certainPixels
            double[] pixel1 = imgHSV.get((int) viewportSize.height/2, (int) viewportSize.width/2);
            double[] pixel2 = imgHSV.get((int) viewportSize.height/2+1, (int) viewportSize.width/2);
            double[] pixel3 = imgHSV.get((int) viewportSize.height/2+1, (int) viewportSize.width/2+1);
            double[] pixel4 = imgHSV.get((int) viewportSize.height/2, (int) viewportSize.width/2+1);

            //Getting Average HSV Values only when Stage=THRESHOLD
            if(gamepad1.options) {
                hHSVAverage = (pixel1[0] + pixel2[0] + pixel3[0] + pixel4[0]) / 4;
                sHSVAverage = (pixel1[1] + pixel2[1] + pixel3[1] + pixel4[1]) / 4;
                vHSVAverage = (pixel1[2] + pixel2[2] + pixel3[2] + pixel4[2]) / 4;
            }

            contoursList.clear();

            //Threshold
            Imgproc.cvtColor(input, imgHSV, Imgproc.COLOR_RGB2HSV);
            Core.extractChannel(imgHSV, HSLchan0Mat, 0);
            Core.extractChannel(imgHSV, HSLchan1Mat, 1);
            Imgproc.threshold(HSLchan0Mat, thresholdMat0, hHSVAverage - rangeAccuracyH, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(HSLchan0Mat, thresholdMat1, hHSVAverage + rangeAccuracyH, 255, Imgproc.THRESH_BINARY_INV);
            Core.bitwise_and(thresholdMat0,thresholdMat1, thresholdMatH);
            Imgproc.threshold(HSLchan1Mat, thresholdMat0, sHSVAverage - rangeAccuracyS, 255, Imgproc.THRESH_BINARY);
            Imgproc.threshold(HSLchan1Mat, thresholdMat1, sHSVAverage + rangeAccuracyS, 255, Imgproc.THRESH_BINARY_INV);
            Core.bitwise_and(thresholdMat0,thresholdMat1, thresholdMatS);

            Core.bitwise_and(thresholdMatH,thresholdMatS, thresholdMat);

            //Find Contours
            Imgproc.findContours(thresholdMat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);
            numContoursFound = contoursList.size();
            if(numContoursFound == 0) {
                rectangleCoordinates[0] = viewportSize.width/2;
                rectangleCoordinates[1] = viewportSize.height/2;
                return input;
            }
            input.copyTo(contoursOnFrameMat);

            //Find the biggest contour
            int contourIdx = 0;
            for(MatOfPoint c : contoursList) {
                if(contoursList.indexOf(c) == 0) {
                    bigContour = c;
                }

                double contourArea = Imgproc.contourArea(contoursList.get(contourIdx));
                if(contourArea > Imgproc.contourArea(bigContour)) bigContour = contoursList.get(contourIdx);
                contourIdx++;
            }

            bigContours.clear();

            //Add Biggest Contour to a list
            bigContours.add(bigContour);

            //Draw the biggest Contour
            Imgproc.drawContours(contoursOnFrameMat, bigContours, -1, new Scalar(0 , 255, 0), 2, 8);

            //Draw Bounding Rectangle

            Imgproc.rectangle(contoursOnFrameMat, Imgproc.boundingRect(bigContour).tl(), Imgproc.boundingRect(bigContour).br(), new Scalar(255,255,0),2);

            int crossThickness = 1;
            double crossSize = 20;

            double[] centerCrossPosition = {0, 0};
            centerCrossPosition[0] = this.viewportSize.width / 2;
            centerCrossPosition[1] = this.viewportSize.height / 2;

            telemetry.addData(">", "Contour Area: " + Imgproc.contourArea(bigContour));

            this.createCross(centerCrossPosition, 1, 12, new Scalar(0, 127, 100), contoursOnFrameMat);

            rectangleCoordinates[0] = (Imgproc.boundingRect(bigContour).br().x - Imgproc.boundingRect(bigContour).tl().x)/2 + Imgproc.boundingRect(bigContour).tl().x;
            rectangleCoordinates[1] = (Imgproc.boundingRect(bigContour).br().y - Imgproc.boundingRect(bigContour).tl().y)/2 + Imgproc.boundingRect(bigContour).tl().y;

            this.createCross(rectangleCoordinates, 1, 8, new Scalar(255, 255, 0), contoursOnFrameMat);

            telemetry.addData(">", "Element's X:" + this.getElementsAnalogCoordinates()[0]);
            telemetry.addData(">", "Element's Y:" + this.getElementsAnalogCoordinates()[1]);
            telemetry.update();

            dashboardTelemetry.addData(">", "Element's X:" + this.getElementsAnalogCoordinates()[0]);
            dashboardTelemetry.update();

            return contoursOnFrameMat;

        }

        public void createCross(double[] pos, int thickness, double sizeOfCross, Scalar color, Mat matToRender) {
            Imgproc.line(
                    matToRender,
                    new Point(pos[0] - sizeOfCross, pos[1]),
                    new Point(pos[0] + sizeOfCross, pos[1]),
                    color,
                    thickness
            );

            Imgproc.line(
                    matToRender,
                    new Point(pos[0], pos[1] - sizeOfCross),
                    new Point(pos[0], pos[1] + sizeOfCross),
                    color,
                    thickness
            );
        }

        public double[] getElementsAnalogCoordinates()
        {
            double[] centerOfScreen = {0, 0};
            double[] rectangleCoordinatesMapped = {0, 0};
            centerOfScreen[0] = viewportSize.width/2;
            centerOfScreen[1] = viewportSize.height/2;
            rectangleCoordinatesMapped[0] = (rectangleCoordinates[0] - centerOfScreen[0]) / (viewportSize.width / 2 + 0.000000000000000000001);
            rectangleCoordinatesMapped[1] = (centerOfScreen[1] - rectangleCoordinates[1]) / (viewportSize.height / 2 + 0.000000000000000000001);

            return rectangleCoordinatesMapped;
        }

        public int getNumContoursFound()
        {
            return numContoursFound;
        }
    }
}