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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class testDriveOP extends LinearOpMode{
    DcMotor frontLeftMotor; // 1
    DcMotor backLeftMotor; // 0
    DcMotor frontRightMotor; // 1 (expansion)
    DcMotor backRightMotor; // 0 (expansion)

    // CRServo clawIntake;
    IMU imu;
    DistanceSensor rightDistanceSensor;
    DistanceSensor backDistanceSensor;

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
        // Hardware Definitions. Must match names setup in robot configuration in the driver hub. config is created and selected selected with driver hub menu
        // Drive Motors
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Chassis-mounted distance sensors
//        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
//        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");

        waitForStart();

        // Set servos to their starting positions
        // clawWrist.setPosition(-1); // start within the starting config
        // rightHang.setPosition(0.6); // Start Closed
        // leftHang.setPosition(0.4);

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
            // if (gamepad1.back) {
            // armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
            // }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
//            double frontLeftPower = (rotY + rotX + rx) / denominator;
//            double backLeftPower = (rotY - rotX + rx) / denominator;
//            double frontRightPower = (rotY - rotX - rx) / denominator;
//            double backRightPower = (rotY + rotX - rx) / denominator;
            double frontLeftPower = (y + x + rx);
            double frontRightPower = (y - x - rx);
            double backLeftPower = (y - x + rx);
            double backRightPower = (y + x - rx);

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower  /= max;
                frontRightPower /= max;
                backLeftPower   /= max;
                backRightPower  /= max;
            }

            if (gamepad1.a) {
                frontLeftMotor.setPower(1);
            }
            else {
                frontLeftMotor.setPower(0);
            }
            if (gamepad1.y) {
                backLeftMotor.setPower(1);
            }
            else {
                backLeftMotor.setPower(0);
            }
            if (gamepad1.x) {
                frontRightMotor.setPower(1);
            }
            else {
                frontRightMotor.setPower(0);
            }
            if (gamepad1.b) {
                backRightMotor.setPower(1);
            }
            else {
                backRightMotor.setPower(0);
            }

            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);


            telemetry.update();
        }
    }
}