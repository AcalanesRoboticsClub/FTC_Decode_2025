// Written by AcaBots FTC team 24689 for the 2025-26 DECODE Season

/*
-------------------- CONTROL SCHEME - CONTROLLER 1 --------------------
Buttons:
    A: Toggle intake, transfer, and riser belts
    START: Reset IMU
    BACK: Reverse intake, transfer, and riser belts + turret intake kicker wheel


D-Pad:
    UP: Close/lobbing preset
    DOWN: Far launch zone preset
    RIGHT: Middle launch zone preset, low angle
    LEFT: Middle launch zone preset, high angle

Triggers:
    RT: Turret rotate right
    LT: Turret rotate left

Shoulder Buttons:
    RB: Turret flywheel
    LB: Turret intake kicker wheel

Joysticks:
    Right: Relative Chassis Rotation
    Left: Absolute Chassis Strafe based on orientation when START button is pressed

-------------------- CONTROL SCHEME - CONTROLLER 2 --------------------
Same as controller 1, except turret rotation is disabled
 */

package org.firstinspires.ftc.teamcode;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp @Config
public class omniTeleOP2 extends LinearOpMode{
    int FLYWHEEL_ROTATE_MAX = 1200;
    int FLYWHEEL_ROTATE_MIN = -1600;
    boolean intakeToggle = false;
    double flywheelVelocitySet = 0;
    DcMotor frontLeftMotor; // 1
    DcMotor backLeftMotor; // 0
    DcMotor frontRightMotor; // 1 (expansion)
    DcMotor backRightMotor; // 0 (expansion)
    CRServo intakeLeft; // 1
    CRServo intakeRight; // 0 (expansion)
    CRServo beltLeft; // 0
    CRServo beltRight; // 1 (expansion)
    CRServo beltVertical; // 2
    DcMotor flywheelRotateMotor; // 2 (expansion)
    DcMotorEx flywheelMotor; // 2
    DcMotor flywheelIntake; // 3
    Servo flywheelAngle; // 2 (expansion)
    Servo ledIndicator;
    IMU imu;
    GoBildaPinpointDriver odo;
    double angle;

    // PID CONTROLLER
    boolean aimingToggle = false;
    boolean blueSideToggle = true;
    double TURRET_TPR = 537.7 * (10.0 / 70.0);   // Example for 5203-312 with 10:70 ratio
    public static class Params {
        public double kP = -2;
        public double kI = 0;
        public double kD = 0;
    }
    public static TurretTestOP.Params PARAMS = new TurretTestOP.Params();
    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    double error = 0;
    Pose2D robotPos;
    double turretAngle = 0.0;
    double cameraHeading;
    double CORNER_ANGLE;
    double integral = 0;
    double pidOutput = 0.0;
    double totalOutput;
    double flywheelMaxAngle;
    double flywheelMinAngle;
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if(Math.abs(b) > Math.abs(a)) {
            return b;
        } else {
            return a;
        }
    }

    public double expo(double input, double exponent) { // for scaled non linear throttle curve driving
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // SET UP CAMERA
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();



        // Hardware Definitions. Must match names setup in robot configuration in the driver hub. config is created and selected selected with driver hub menu
        // Drive Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Game Element Intake
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        // Belts to connect intake and flywheel
        beltLeft = hardwareMap.get(CRServo.class, "beltLeft");
        beltRight = hardwareMap.get(CRServo.class, "beltRight");
        beltVertical = hardwareMap.get(CRServo.class, "beltVertical");

        ledIndicator = hardwareMap.get(Servo.class, "led"); // RGB Led headlight/debug light

        // Reverse some belts
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        beltLeft.setDirection(CRServo.Direction.REVERSE);

        // Flywheel Motor and Rotation
        flywheelRotateMotor = hardwareMap.dcMotor.get("rotatShot");
        flywheelRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheelMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelIntake = hardwareMap.dcMotor.get("flywheelIntake");
        flywheelAngle = hardwareMap.get(Servo.class, "flywheelAngle");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Setup odometry params
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(0, 0, DistanceUnit.INCH); // <-------------------------------------------- NO IDEA WHAT THIS DOES AT ALL
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize odometry stuff when match starts
        odo.resetPosAndIMU();

        // For RoadRunner pathing
        odo.setHeading(0, AngleUnit.DEGREES); // Set initial angle
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0)); // starting coordinates and heading
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        timer.reset();
        double previousRobotHeading = odo.getHeading(AngleUnit.RADIANS);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            odo.update();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.initialize(parameters);
                imu.resetYaw();
                odo.setHeading(0, AngleUnit.DEGREES);
                // odo.setPosition(new Pose2d(0, 0, Math.toRadians(0)));
                // odo.resetPosAndIMU();
                aimingToggle = true;
            }

            // ===== Adjustable expo values =====
            double driveExpo = 1.5;   // 1.0 = linear, 2.0 = quadratic, 3.0 = very soft center
            double turnExpo = 1.5;


            // ===== Read & scale driver input =====
            double x = expo(calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x), driveExpo);
            double y = expo(calcLargestChange(-gamepad1.left_stick_y, -gamepad2.left_stick_y), driveExpo); // negative = forward
            double rx = expo(calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x), turnExpo);

            // ===== Field-centric math =====
            double botHeading = -odo.getHeading(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            rotX *= 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            // ===== Normalize =====
            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // ===== Apply power =====
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);


            // Controller 1 Intake
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                intakeToggle = !intakeToggle;
            }

            if (intakeToggle) {
                intakeLeft.setPower(1);
                intakeRight.setPower(1);
                beltLeft.setPower(1);
                beltRight.setPower(1);
                beltVertical.setPower(-1);

            } else if (gamepad1.back || gamepad2.back) { // reverse all intake + flywheel intake in emergency
                intakeLeft.setPower(-1);
                intakeRight.setPower(-1);
                beltLeft.setPower(-1);
                beltRight.setPower(-1);
                beltVertical.setPower(1);
                flywheelIntake.setPower(-1);
            } else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
                beltLeft.setPower(0);
                beltRight.setPower(0);
                beltVertical.setPower(0);
            }


            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                //flywheelMotor.setPower(1 * flywheelSpeedMultiplier);
                flywheelMotor.setVelocity(flywheelVelocitySet);
            } else {
                flywheelMotor.setPower(0);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                flywheelIntake.setPower(1);
            } else {
                flywheelIntake.setPower(0);
            }

            // Control the indicator light
            if (flywheelMotor.getVelocity() > flywheelVelocitySet - 100) // under speed
            {
                ledIndicator.setPosition(0.5); // green
            } else if (flywheelVelocitySet != 0) {
                ledIndicator.setPosition(0.28); // red
            }

            if (gamepad1.dpad_up || gamepad2.dpad_up) { // CLOSEST (touching wall)
                angle = 0;
                flywheelVelocitySet = 1280; // 1500
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) { // CLOSE (centered on closer triangle)
                angle = 0.26;
                flywheelVelocitySet = 1350; // 1800
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) { // FAR (centered on top of triangle)
                angle = 0.23;
                flywheelVelocitySet = 1800; // 2200
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) { // CLOSE (other setting)
                angle = 0.26;
                flywheelVelocitySet = 1550; // 1920
            }
            // angle is between 0 and 0.4
            flywheelAngle.setPosition(angle);
            telemetry.addData("flywheel position: ", flywheelAngle.getPosition());

            // FLYWHEEL AUTO-AIMING
            if (gamepad1.bWasPressed() || gamepad2.bWasPressed()) {
                aimingToggle = !aimingToggle;
            }
            if (gamepad1.xWasPressed() || gamepad2.xWasPressed()) {
                blueSideToggle = true;
            }
            if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {
                blueSideToggle = false;
            }

            List<AprilTagDetection> rawDetections = tagProcessor.getDetections();
            telemetry.addData("data", tagProcessor.getDetections());
            double robotHeading = 0.0;
            if (flywheelRotateMotor.getCurrentPosition() >= FLYWHEEL_ROTATE_MAX) {
                flywheelRotateMotor.setPower(-0.2);
            } else if (flywheelRotateMotor.getCurrentPosition() <= FLYWHEEL_ROTATE_MIN) {
                flywheelRotateMotor.setPower(0.2);
            } else if (aimingToggle == true) {
                double dt = timer.seconds();
                timer.reset();

                // Get robot heading and position
                robotHeading = -odo.getHeading(AngleUnit.RADIANS);                // radians
                if (robotHeading > 3.0) {
                    robotHeading = robotHeading - 2 * Math.PI;
                }
                previousRobotHeading = robotHeading;
                robotPos = odo.getPosition();
                turretAngle = -flywheelRotateMotor.getCurrentPosition() / TURRET_TPR * Math.PI / 23;

                cameraHeading = robotHeading + turretAngle;
                CORNER_ANGLE = 0.0;
                if (blueSideToggle) {
                    CORNER_ANGLE = Math.atan2(72 + robotPos.getY(DistanceUnit.INCH), 72 - robotPos.getX(DistanceUnit.INCH));
                }
                else {
                    CORNER_ANGLE = Math.atan2(72 + robotPos.getY(DistanceUnit.INCH), 72 + robotPos.getX(DistanceUnit.INCH));
                }

                error = cameraHeading - CORNER_ANGLE;
                double OFFSET_ANGLE = -0.06;
                error += OFFSET_ANGLE;

                integral += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;

                pidOutput = PARAMS.kP * error +
                        PARAMS.kI * integral +
                        PARAMS.kD * derivative;

                totalOutput = 0;
                flywheelMaxAngle = FLYWHEEL_ROTATE_MAX / TURRET_TPR * Math.PI / 23;
                flywheelMinAngle = FLYWHEEL_ROTATE_MIN / TURRET_TPR * Math.PI / 23;
                if (CORNER_ANGLE <= 0) {
                    totalOutput = -pidOutput;
                } else if (CORNER_ANGLE > 0) {
                    totalOutput = pidOutput;
                }
                if ((totalOutput < 0 && cameraHeading + error < flywheelMinAngle) && (totalOutput > 0 && cameraHeading + error > flywheelMaxAngle)) {
                    totalOutput = -totalOutput;
                }
                flywheelRotateMotor.setPower(-totalOutput);
            }
            else {
                if ((gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1)
                        && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {

                    double expo = 1.5; // 1 = linear, 2 = quadratic, 3 = cubic
                    double power = Math.pow(gamepad1.left_trigger, expo);

                    flywheelRotateMotor.setPower(power * power);

                } else if ((gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1)
                        && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {

                    double expo = 1.5; // 1 = linear, 2 = quadratic, 3 = cubic
                    double power = Math.pow(gamepad1.right_trigger, expo);
                    flywheelRotateMotor.setPower(-(power * power));

                } else {
                    flywheelRotateMotor.setPower(0);
                }
            }

            telemetry.addData("flyweelRotate: ", flywheelRotateMotor.getPower());
            telemetry.addData("aimingToggle:", aimingToggle);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("error", error);
            packet.put("kP", PARAMS.kP);
            packet.put("kI", PARAMS.kI);
            packet.put("kD", PARAMS.kD);
            packet.put("kF", PARAMS.kD);
            packet.put("turret angle", turretAngle);
            packet.put("robot angle", robotHeading);
            packet.put("camera heading", cameraHeading);
            packet.put("robot position x", robotPos.getX(DistanceUnit.INCH));
            packet.put("robot position y", robotPos.getY(DistanceUnit.INCH));
            packet.put("corner angle", CORNER_ANGLE);
            packet.put("flywheel max angle", flywheelMaxAngle);
            packet.put("flywheel min angle", flywheelMinAngle);
            packet.put("turretPos", flywheelRotateMotor.getCurrentPosition());
            packet.put("integral", integral);
            packet.put("pidOutput", pidOutput);
            packet.put("totalOutput", totalOutput);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.addData("rmp flywheel: ", flywheelMotor.getPower());
            telemetry.update();
        }
    }
}