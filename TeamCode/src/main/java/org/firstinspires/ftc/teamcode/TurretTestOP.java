// Written primarily by Henry Rosenberg for AcaBots, FTC Team #24689
// hi also sze ting
/*
-------------------- CONTROL SCHEME - CONTROLLER 1 --------------------
Buttons:
    A:
    B:
    X:
    Y:

D-Pad:
    UP:
    DOWN:
    RIGHT:

Triggers:
    RT: Hold for Intake
    LT: Hold for Reverse Intake
    BOTH:

Shoulder Buttons:
    RB:
    LB:

Joysticks:
    Right: Relative Chassis Rotation
    Left: Absolute Chassis Strafe based on orientation when START button is pressed

-------------------- CONTROL SCHEME - CONTROLLER 2 --------------------
Buttons:
    A: Hold for Flywheel
    B: Hold for Reverse Flywheel
    Y:

Triggers:
    RT: Hold for Belts
    LT: Hold for Reverse Belts

Joysticks:
    Right: Relative Chassis Rotation
    Left: Absolute Chassis Strafe based on orientation when START button is pressed

 ---------------------------- START CONFIG ----------------------------
 Hanging Hooks: Open
 Arm Slide: Retracted
 Arm Pivot: Down, resting on bottom stop
 Claw Wrist: Folded left
 */

package org.firstinspires.ftc.teamcode;
import android.util.Size;

//import com.arcrobotics.ftclib.controller.PIDController;
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

/*
DRIVER SCHEMA:
- Driver 1
    chassis driver
    flywheel rotation
- Driver 2
    flywheel shooting and presets
    intake
*/

@TeleOp @Config
public class TurretTestOP extends LinearOpMode{
    public static double normDelta(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    int FLYWHEEL_ROTATE_MAX = 1300;
    int FLYWHEEL_ROTATE_MIN = -1750;
    double SLOWER_SPEED_MULTIPLIER = 0.77;
    boolean intakeToggle = false;
    boolean toggleSlowerShoot = false;
    double flywheelSpeedMultiplier = 1.0;

    DcMotor frontLeftMotor; // 1
    DcMotor backLeftMotor; // 0
    DcMotor frontRightMotor; // 1 (expansion)
    DcMotor backRightMotor; // 0 (expansion)

    DcMotor flywheelRotateMotor; // 2 (expansion)
    DcMotor flywheelMotor; // 2
    DcMotor flywheelIntake; // 3

    Servo flywheelAngle; // 2 (expansion)
    // CRServo clawIntake;
    IMU imu;
    GoBildaPinpointDriver odo;
    DistanceSensor rightDistanceSensor;
    DistanceSensor backDistanceSensor;
    double angle;

    // PID CONTROLLER
    // --- Turret constants ---
    double TURRET_TPR = 537.7 * (10.0 / 70.0);   // Example for 5203-312 with 10:70 ratio
    public static class Params {
        public double kP = -2;
        public double kI = -5;
        public double kD = 0;
        public double kF = 0;
    }
    public static Params PARAMS = new Params();
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
    double yaw = 0;
    double targetYaw = 0;

    // 1D Kalman Filter
    KalmanFilter1D yawFilter = new KalmanFilter1D(0.01, 2.0);

    //    PIDController pid = new PIDController(kP, kI, kD);
    FtcDashboard dashboard = FtcDashboard.getInstance();

    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if(Math.abs(b) > Math.abs(a)) {
            return b;
        } else {
            return a;
        }
    }

    private int setSignFromReference(int newAbsoluteValue, int signReference) {
        // Return the value of newAbsoluteValue with the + or - sign of signReference. Used for teleOp presets
        if (signReference <= 0) {
            return -newAbsoluteValue;
        } else {
            return newAbsoluteValue;
        }
    }

//    @Override
//    public boolean run(@NonNull TelemetryPacket p) {
//        p.put("yaw: ", yaw);
//        p.put("target yaw: ", targetYaw);
//        p.put("error: ", yawError);
//
//        // only draw when active; only one drive action should be active at a time
//        Canvas c = p.fieldOverlay();
//
//        c.setStroke("#4CAF50");
//        Drawing.drawRobot(c, Pose2d());
//
//        c.setStroke("#3F51B5");
//        Drawing.drawRobot(c, targetYaw);
//
//        c.setStroke("#4CAF50FF");
//        c.setStrokeWidth(1);
//        c.strokePolyline(xPoints, yPoints);
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        // SET UP CAMERA
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // SET UP ODOMETRY
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(0, 0, DistanceUnit.INCH); // <-------------------------------------------- NO IDEA WHAT THIS DOES AT ALL
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize odometry stuff when match starts
        odo.resetPosAndIMU();

        // For RoadRunner pathing
        odo.setHeading(0, AngleUnit.DEGREES); // Set initial angle (0 or 270?)
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(270)); // starting coordinates and heading
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // SET UP HARDWARE
        // Hardware Definitions. Must match names setup in robot configuration in the driver hub. config is created and selected selected with driver hub menu
        // Drive Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Flywheel Motor and Rotation
        flywheelRotateMotor = hardwareMap.dcMotor.get("rotatShot");
        flywheelRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        flywheelIntake = hardwareMap.dcMotor.get("flywheelIntake");
        flywheelAngle = hardwareMap.get(Servo.class, "flywheelAngle");

        // SET UP IMU
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1920, 1080))
                .build();

        waitForStart();

        timer.reset();
        double previousRobotHeading = odo.getHeading(AngleUnit.RADIANS);

        while (!isStopRequested() && opModeIsActive()) {
            // test to find ratio - do later
//            telemetry.addData("ticks", flywheelRotateMotor.getCurrentPosition());

            odo.update();

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.initialize(parameters);
                imu.resetYaw();
                odo.setHeading(0, AngleUnit.DEGREES);
            }

            // CHASSIS DRIVING
            // Take whichever value is the most drastic change to use from either controller
            double y = calcLargestChange(-gamepad1.left_stick_y, -gamepad2.left_stick_y); // Y stick values are reported as inverted by the controller
            double x = calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x);
            double rx = calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x);

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Flywheel
            if (toggleSlowerShoot) {
                flywheelSpeedMultiplier = SLOWER_SPEED_MULTIPLIER;
            } else {
                flywheelSpeedMultiplier = 1;
            }

            if (gamepad1.right_bumper || gamepad2.right_bumper) {
                flywheelMotor.setPower(1 * flywheelSpeedMultiplier);
            } else {
                flywheelMotor.setPower(0);
            }

            if (gamepad1.left_bumper || gamepad2.left_bumper) {
                flywheelIntake.setPower(1);
            } else {
                flywheelIntake.setPower(0);
            }

            if (gamepad1.dpad_up || gamepad2.dpad_up) { // CLOSEST (touching wall)
                angle = 0;
                toggleSlowerShoot = true;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) { // CLOSE (centered on closer triangle)
                angle = 0.3;
                toggleSlowerShoot = false;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) { // FAR (centered on top of triangle)
                angle = 0.25;
                toggleSlowerShoot = false;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) { // CLOSE (other setting)
                angle = 0.34;
                toggleSlowerShoot = false;
            }
            // angle is between 0 and 0.4
            flywheelAngle.setPosition(angle);
            telemetry.addData("flywheel position: ", flywheelAngle.getPosition());

            // FLYWHEEL AUTO-AIMING
            List<AprilTagDetection> rawDetections = tagProcessor.getDetections();
            telemetry.addData("data", tagProcessor.getDetections());
            double robotHeading = 0.0;
            if (flywheelRotateMotor.getCurrentPosition() >= FLYWHEEL_ROTATE_MAX) {
                flywheelRotateMotor.setPower(-0.2);
            }
            else if (flywheelRotateMotor.getCurrentPosition() <= FLYWHEEL_ROTATE_MIN) {
                flywheelRotateMotor.setPower(0.2);
            }
//            else if (!(rawDetections.size() == 0) && !(rawDetections == null)) {
//                double dt = timer.seconds();
//                timer.reset();
//
//                // --- ODOMETRY: robot heading + heading rate ---
//                double robotHeading = odo.getHeading(AngleUnit.DEGREES);                // radians
//                double robotHeadingRate = (robotHeading - previousRobotHeading) / dt;
//                previousRobotHeading = robotHeading;
//
//                Pose2D robotPos = odo.getPosition();
//                turretAngle = flywheelRotateMotor.getCurrentPosition() / TURRET_TPR;
//
//                double cameraHeading = robotHeading + turretAngle;
//
//                AprilTagDetection tag = rawDetections.get(0);
//                double rawYawDeg = tag.ftcPose.yaw;
//                double rawYaw = Math.toRadians(rawYawDeg);
//                double filteredYaw = yawFilter.update(rawYaw);
//
//                // initialize on first frame
//                if (!lpfInit) {
//                    prevRawYaw = rawYaw;
//                    yaw = 0;          // HPF starts at 0
//                    lpfInit = true;
//                }
//
//                // ----- High-pass filter -----
//                double highPassYaw = ALPHA * (yaw + (filteredYaw - prevRawYaw));
//
//                // update stored values
//                prevRawYaw = filteredYaw;
//
//                // use high-pass signal
//                yaw = -highPassYaw;
//
//                // --- Combined error ---
//                double CORNER_ANGLE = Math.atan2(robotPos.getY(DistanceUnit.INCH), robotPos.getX(DistanceUnit.INCH));
//                error = cameraHeading - CORNER_ANGLE;
//
//                // Normalize angle to (-PI, PI)
//                // error = Math.atan2(Math.sin(error), Math.cos(error));
//
//                // --- PID controller ---
//                integral += error * dt;
//                double derivative = (error - lastError) / dt;
//                lastError = error;
//
//                pidOutput = PARAMS.kP * error +
//                        PARAMS.kI * integral +
//                        PARAMS.kD * derivative;
//
//                flywheelRotateMotor.setPower(pidOutput);
//
//                telemetry.addData("error", error);
//                telemetry.addData("PID Output", pidOutput);
//            }
            else {
                double dt = timer.seconds();
                timer.reset();

                // Get robot heading and position
                robotHeading = -odo.getHeading(AngleUnit.RADIANS);                // radians
                if (robotHeading > 3.0) {
                    robotHeading = robotHeading - 2*Math.PI;
                }
                previousRobotHeading = robotHeading;
                robotPos = odo.getPosition();
                turretAngle = -flywheelRotateMotor.getCurrentPosition() / TURRET_TPR * Math.PI / 23;

                cameraHeading = robotHeading + turretAngle;
                CORNER_ANGLE = Math.atan2(72 + robotPos.getY(DistanceUnit.INCH), 72 - robotPos.getX(DistanceUnit.INCH));

                error = cameraHeading - CORNER_ANGLE;

                integral += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;

                pidOutput = PARAMS.kP * error +
                        PARAMS.kI * integral +
                        PARAMS.kD * derivative;

                totalOutput = 0;
                if (CORNER_ANGLE <= 0) {
                    totalOutput = -pidOutput;
                }
                else if (CORNER_ANGLE > 0) {
                    totalOutput = pidOutput;
                }
                flywheelRotateMotor.setPower(-totalOutput);

            }
//                else {
//                        yawError = -lastError;
//                        integralSum += yawError * timer.seconds();
//                        double derivative = (yawError - lastError) / timer.seconds();
////                    lastError = yawError;
//
//                        timer.reset();
//
//                        double pidOutput = (yawError * PARAMS.kP) + (derivative * PARAMS.kD) + (integralSum * PARAMS.kI);
//                        flywheelRotateMotor.setPower(-pidOutput);
//                    telemetry.addData("error",e yawError);
//                    telemetry.addData("PID Output", pidOutput);
//                }
            telemetry.addData("flyweelRotate: ", flywheelRotateMotor.getPower());

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
            packet.put("turretPos", flywheelRotateMotor.getCurrentPosition());
            packet.put("integral", integral);
            packet.put("pidOutput", pidOutput);
            packet.put("totalOutput", totalOutput);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

//                flywheelRotateMotor.setPower(pid.calculate(flywheelRotateMotor.getCurrentPosition(), setpoint));

            //            telemetry.addData("Right Distance (mm): ", rightDistanceSensor.getDistance(DistanceUnit.MM));
            //            telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
            telemetry.addData("rmp flywheel: ", flywheelMotor.getPower());
            telemetry.update();
        }
    }
}

class KalmanFilter1D {
    private double Q;  // Process noise covariance
    private double R;  // Measurement noise covariance
    private double x;  // State estimate
    private double P;  // Estimate covariance
    private boolean initialized = false;

    public KalmanFilter1D(double Q, double R) {
        this.Q = Q;   // small
        this.R = R;   // bigger
    }

    public double update(double measurement) {
        if (!initialized) {
            x = measurement;
            P = 1;
            initialized = true;
            return x;
        }

        // Prediction step
        P = P + Q;

        // Kalman Gain
        double K = P / (P + R);

        // Update step
        x = x + K * (measurement - x);

        // Update covariance
        P = (1 - K) * P;

        return x;
    }
}

//if (tag.ftcPose.yaw < -4 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
//    flywheelRotateMotor.setPower(-0.5);
//}
//else if (tag.ftcPose.yaw < 4 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
//    flywheelRotateMotor.setPower(-0.3);
//}
//else if (tag.ftcPose.yaw < 6.8 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
//    flywheelRotateMotor.setPower(-0.1);
//}
//if (tag.ftcPose.yaw < 7.8 && tag.ftcPose.yaw > 6.8) {
//    flywheelRotateMotor.setPower(0);
//}

//AprilTagDetection tag = tagProcessor.getDetections().get(0);
//double yaw = tag.ftcPose.yaw;   // yaw (left/right) of camera
//double targetYaw = 0.0;         // want to be centered on tag
//double error = yaw - targetYaw;
//
//integralSum += error * dt;
//double derivative = (error - lastError) / dt;
//double pidOutput =
//        (error * PARAMS.kP) +
//                (derivative * PARAMS.kD) +
//                (integralSum * PARAMS.kI);
//
//                flywheelRotateMotor.setPower(-pidOutput);


//
//// angle of turret and robots
//double TICKS_PER_REV_TURRET = 537.7;  // * 7.2?
//double turretAngle = (flywheelRotateMotor.getCurrentPosition() / TICKS_PER_REV_TURRET) * 360.0;
//angleDiff = turretAngle - robotAngle;
//double actualTurretWorldAngle = turretAngle + robotAngle;
//
//robotAngle = odo.getHeading(AngleUnit.DEGREES);
//double robotAngularVelocity = normDelta(robotAngle - lastRobotAngle) / timer.seconds();
//lastRobotAngle = robotAngle;
//
//
//AprilTagDetection tag = rawDetections.get(0);
//double rawYaw = tag.ftcPose.yaw;
//double filteredYaw = yawFilter.update(rawYaw);
//// initialize filter on first frame
//                if (!lpfInit) {
//lpfYaw = filteredYaw;
//lpfInit = true;
//        }
//
//// Low-pass filter update
//lpfYaw = lpfYaw + ALPHA * (filteredYaw - lpfYaw);
//
//// use filtered yaw
//yaw = -lpfYaw;
//targetYaw = 0.0;
////                yawError = targetYaw - yaw;
//double targetTurretWorldAngle = robotAngle + yaw;
////                double targetWorldYaw = robotAngle - yaw;   // negative because tag yaw flips direction
//error = normDelta(targetTurretWorldAngle - actualTurretWorldAngle);
//
//integralSum += error * timer.seconds();
//double derivative = (error - lastError) / timer.seconds();
//lastError = error;
//
//double actual_kD = PARAMS.kD;
//
//                if ((frontLeftMotor.getPower() > 0.3 && backLeftMotor.getPower() > 0.3) ||
//        (frontRightMotor.getPower() > 0.3 && backRightMotor.getPower() > 0.3)) {
//actual_kD *= 1.2;
//        }
//        telemetry.addData("actual kD", actual_kD);
//
//                timer.reset();
//
//pidOutput = (error * PARAMS.kP) + (derivative * actual_kD) + (integralSum * PARAMS.kI);
//double turretFeedforward = -PARAMS.kF * robotAngularVelocity;
//double finalOutput = pidOutput + turretFeedforward;
//                flywheelRotateMotor.setPower(-finalOutput);
