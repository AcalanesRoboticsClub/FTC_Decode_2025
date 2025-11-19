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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class omniTeleOP extends LinearOpMode{
    int FLYWHEEL_ROTATE_MAX = 1350;
    int FLYWHEEL_ROTATE_MIN = -2150;
    boolean intakeToggle = false;
    double flywheelSpeedMultiplier = 1.0;
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
    DcMotor flywheelMotor; // 2
    DcMotor flywheelIntake; // 3
    Servo flywheelAngle; // 2 (expansion)
    IMU imu;
    double angle;

    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if(Math.abs(b) > Math.abs(a)) {
            return b;
        } else {
            return a;
        }
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

        // Reverse some belts
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        beltLeft.setDirection(CRServo.Direction.REVERSE);

        // Flywheel Motor and Rotation
        flywheelRotateMotor = hardwareMap.dcMotor.get("rotatShot");
        flywheelRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        flywheelIntake = hardwareMap.dcMotor.get("flywheelIntake");
        flywheelAngle = hardwareMap.get(Servo.class, "flywheelAngle");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // Take whichever value is the most drastic change to use from either controller
            double y = calcLargestChange(-gamepad1.left_stick_y, -gamepad2.left_stick_y); // Y stick values are reported as inverted by the controller
            double x = calcLargestChange(gamepad1.left_stick_x, gamepad2.left_stick_x);
            double rx = calcLargestChange(gamepad1.right_stick_x, gamepad2.right_stick_x);

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.options) {
                imu.initialize(parameters);
                imu.resetYaw();
            }

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

            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);


            // Controller 1 Intake
            if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {
                intakeToggle = !intakeToggle;
            }

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
                flywheelSpeedMultiplier = 0.66;
            }
            if (gamepad1.dpad_left || gamepad2.dpad_left) { // CLOSE (centered on closer triangle)
                angle = 0.28;
                flywheelSpeedMultiplier = 0.82;
            }
            if (gamepad1.dpad_down || gamepad2.dpad_down) { // FAR (centered on top of triangle)
                angle = 0.23;
                flywheelSpeedMultiplier = 0.92;
            }
            if (gamepad1.dpad_right || gamepad2.dpad_right) { // CLOSE (other setting)
                angle = 0.28;
                flywheelSpeedMultiplier = 0.76;
            }
            // angle is between 0 and 0.4
            flywheelAngle.setPosition(angle);
            telemetry.addData("flywheel position: ", flywheelAngle.getPosition());

            // FLYWHEEL AUTO-AIMING
            telemetry.addData("data", tagProcessor.getDetections());
            if (!tagProcessor.getDetections().isEmpty()) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("yaw", tag.ftcPose.yaw);
            }
            if (!tagProcessor.getDetections().isEmpty() && (gamepad1.b || gamepad2.b)) {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("id", tag.id);
                if (tag.id == 20 || tag.id == 19) {
                    telemetry.addData("x", tag.ftcPose.x);
                    telemetry.addData("y", tag.ftcPose.y);
                    telemetry.addData("z", tag.ftcPose.z);
                    telemetry.addData("roll", tag.ftcPose.roll);
                    telemetry.addData("pitch", tag.ftcPose.pitch);
                    telemetry.addData("yaw", tag.ftcPose.yaw);

                    // yaw can be like -50 to 50
                    if (tag.ftcPose.yaw > 15 && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {
                        flywheelRotateMotor.setPower(0.5);
                    }
                    else if (tag.ftcPose.yaw > 10 && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {
                        flywheelRotateMotor.setPower(0.3);
                    }
                    else if (tag.ftcPose.yaw > 7.8 && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {
                        flywheelRotateMotor.setPower(0.1);
                    }
                    if (tag.ftcPose.yaw < -4 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
                        flywheelRotateMotor.setPower(-0.5);
                    }
                    else if (tag.ftcPose.yaw < 4 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
                        flywheelRotateMotor.setPower(-0.3);
                    }
                    else if (tag.ftcPose.yaw < 6.8 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
                        flywheelRotateMotor.setPower(-0.1);
                    }
                    if (tag.ftcPose.yaw < 7.8 && tag.ftcPose.yaw > 6.8) {
                        flywheelRotateMotor.setPower(0);
                    }
                }
            }
            else {
                if (!(gamepad1.b || gamepad2.b) && (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {
                    flywheelRotateMotor.setPower(gamepad1.left_trigger * 0.85);
                } else if (!(gamepad1.b || gamepad2.b) && (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
                    flywheelRotateMotor.setPower(gamepad1.right_trigger * -0.85);
                } else {
                    flywheelRotateMotor.setPower(0);
                }
                telemetry.addData("flywheel rotate: ", flywheelRotateMotor.getCurrentPosition());
            }
            telemetry.addData("flyweelRotate power: ", flywheelRotateMotor.getPower());
            telemetry.addData("flywheel power: ", flywheelMotor.getPower());
            telemetry.update();
        }
    }
}