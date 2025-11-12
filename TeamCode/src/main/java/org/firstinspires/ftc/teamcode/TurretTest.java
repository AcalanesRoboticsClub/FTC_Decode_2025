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

import com.arcrobotics.ftclib.controller.PIDController;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
DRIVER SCHEMA:
- Driver 1
    chassis driver
    flywheel rotation
- Driver 2
    flywheel shooting and presets
    intake
*/

@TeleOp
public class TurretTest extends LinearOpMode{
    int FLYWHEEL_ROTATE_MAX = 1350;
    int FLYWHEEL_ROTATE_MIN = -2150;
    double SLOWER_SPEED_MULTIPLIER = 0.77;
    boolean intakeToggle = false;
    boolean toggleSlowerShoot = false;
    double flywheelSpeedMultiplier = 1.0;

    DcMotor flywheelRotateMotor; // 2 (expansion)
    DcMotor flywheelMotor; // 2
    DcMotor flywheelIntake; // 3

    Servo flywheelAngle; // 2 (expansion)
    // CRServo clawIntake;
    IMU imu;
    DistanceSensor rightDistanceSensor;
    DistanceSensor backDistanceSensor;
    double angle;
    int setpoint = 0;

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

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {

            // Flywheel Motor and Rotation
            flywheelRotateMotor = hardwareMap.dcMotor.get("rotatShot");
            flywheelRotateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            flywheelRotateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
            flywheelIntake = hardwareMap.dcMotor.get("flywheelIntake");
            flywheelAngle = hardwareMap.get(Servo.class, "flywheelAngle");

            // Retrieve the IMU from the hardware map
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
            IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));
            // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
            imu.initialize(parameters);

            // Chassis-mounted distance sensors
            //        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
            //        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");

            waitForStart();


            if (isStopRequested()) return;

            while (opModeIsActive()) {

                // PID CONTROLLER
                int kP = 0;
                int kI = 0;
                int kD = 0;
                PIDController pid = new PIDController(kP, kI, kD);


                // This button choice was made so that it is hard to hit on accident,
                // it can be freely changed based on preference.
                // The equivalent button is start on Xbox-style controllers.
                if (gamepad1.options) {
                    imu.initialize(parameters);
                    imu.resetYaw();
                }

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

                        if (tag.ftcPose.yaw > 7.8 && flywheelRotateMotor.getCurrentPosition() < FLYWHEEL_ROTATE_MAX) {
                            setpoint += 10;
                        }
                        else if (tag.ftcPose.yaw < 6.8 && flywheelRotateMotor.getCurrentPosition() > FLYWHEEL_ROTATE_MIN) {
                            setpoint -= 10;
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
                telemetry.addData("flyweelRotate: ", flywheelRotateMotor.getPower());

                flywheelRotateMotor.setPower(pid.calculate(flywheelRotateMotor.getCurrentPosition(), setpoint));

                //            telemetry.addData("Right Distance (mm): ", rightDistanceSensor.getDistance(DistanceUnit.MM));
                //            telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.addData("rmp flywheel: ", flywheelMotor.getPower());
                telemetry.update();
            }
        }
    }
}