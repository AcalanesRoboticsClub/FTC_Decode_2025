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
public class omniTeleOP extends LinearOpMode{
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    CRServo bottomBelt;
    CRServo angledBelt;
    CRServo intakeLeft;
    CRServo intakeRight;
    DcMotor flywheelMotor;
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

        // Arm Pivot Motor
        // Encoder of ~2500 is vertical
        // armPivotMotor = hardwareMap.dcMotor.get("armPivotMotor");
        // armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
        // int armPivotDesiredPos = 400; // position the arm pivot will assume when the program is run, until a different position is commanded

        // Arm Slide Motor
        // Encoder of |2130| is fully extended
        // armSlideMotor = hardwareMap.dcMotor.get("armSlideMotor");
        // armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks
        // int armSlideDesiredPos = 0;
        // boolean armSlideLastMoveDirection = false; // false = retract, true = extend, starting is equivalent to having just retracted
        // double armSlideHoldingPower = 0.5; // Power to hold slide in place, 0.5 to prevent overheating, can be set to 1 for hangs
        // int armSlideSoftLimit = 2000;

        // Hanging Claws
        // rightHang = hardwareMap.get(Servo.class, "rightHangServo");
        // leftHang = hardwareMap.get(Servo.class, "leftHangServo");

        // Game Element Intake
//        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
//        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");

        // Belts to connect intake and flywheel
//        bottomBelt = hardwareMap.get(CRServo.class, "bottomBelt");
//        angledBelt = hardwareMap.get(CRServo.class, "angledBelt");

        // Flywheel Motor
//        flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");

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

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            telemetry.addData("vals", "%4.2f, %4.2f, %4.2f", y, x, rx);
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);

            // Controller 1 Arm Pivot Motor
            /*
            int armPivotPos = armPivotMotor.getCurrentPosition(); // current position of the slide, used to prevent overextension/going past 0
            if((gamepad1.right_trigger > 0.3 && armPivotPos <= 4000) && gamepad1.left_trigger < 0.1) {
                armPivotMotor.setPower(gamepad1.right_trigger); // extend at the power of the trigger
                armPivotMotor.setDirection(DcMotor.Direction.FORWARD);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivotDesiredPos = armPivotMotor.getCurrentPosition();
            } else if ((gamepad1.left_trigger > 0.3 && (armPivotPos >= 150 || armPivotPos <= -150)) && gamepad1.right_trigger < 0.1) { // encoder pos is inverted when in reverse; so it just checks to make sure it isn't within 50 of zero
                armPivotMotor.setPower(gamepad1.left_trigger);  // retract continuously at the power of the trigger
                armPivotMotor.setDirection(DcMotor.Direction.REVERSE);
                armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armPivotDesiredPos = armPivotMotor.getCurrentPosition();
            } else {
                armPivotMotor.setTargetPosition(armPivotDesiredPos); // hold the motor in its current position
                armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Use builtin PID loop to hold position
                armPivotMotor.setPower(1); // Holding power
            }
            */

            // Controller 1 Hanging hooks & arm slide power 'hanging mode'
            /*
            if (gamepad1.x && ((rightHang.getPosition() > 0.55) || (leftHang.getPosition() < 0.45))) { // if A button is pressed AND both of the claws is closed, open the claws
                rightHang.setPosition(0); // They are facing away from each-other, so they start at opposite ends
                leftHang.setPosition(1);
                armSlideHoldingPower = 1; // claws will only be opened for climbing, and full slide power is needed for hanging
                armSlideSoftLimit = 2250; // max reach
                armSlideMotor.setPower(armSlideHoldingPower);
            } else if (gamepad1.x && ((rightHang.getPosition() < 0.55) || (leftHang.getPosition() > 0.45))) { // if A button is pressed AND both of the claws is open, close the claws
                rightHang.setPosition(0.6); // +0.6 from open
                leftHang.setPosition(0.4); // -0.6 from open, due to facing opposite direction
                armSlideHoldingPower = 0.5; // When the claws are closed, there will be no hanging force on the slide
                armSlideSoftLimit = 2000;
                armSlideMotor.setPower(armSlideHoldingPower);
            }
            */

            /*if (gamepad1.dpad_right) {
                armSlideSoftLimit = 5000; // No limit, in case of belt slip
            }*/

            // Controller 1 Intake
            /*if (gamepad1.right_bumper) {
                intakeLeft.setPower(1);
                intakeRight.setPower(-1);
            } else if (gamepad1.left_bumper) {
                intakeLeft.setPower(-1);
                intakeRight.setPower(1);
            }
            else {
                intakeLeft.setPower(0);
                intakeRight.setPower(0);
            }

            // Controller 2 Belts
            if (gamepad2.right_bumper) {
                bottomBelt.setPower(1);
                angledBelt.setPower(1);
            }
            else if (gamepad2.left_bumper) {
                bottomBelt.setPower(-1);
                angledBelt.setPower(-1);
            }
            else {
                bottomBelt.setPower(0);
                angledBelt.setPower(0);
            }

            // Controller 2 Flywheel
            if (gamepad2.a) {
                flywheelMotor.setPower(10);
            }
            if (gamepad2.b) {
                flywheelMotor.setPower(-10);
            }
            else {
                flywheelMotor.setPower(0);
            }*/

            // Controller 1 Arm Slide
            // encoder directions become negative depending on motor directions
            /*int armSlidePos = armSlideMotor.getCurrentPosition(); // current position of the slide, used to prevent overextension/going past 0
            if(gamepad1.dpad_up && (Math.abs(armSlidePos) <= armSlideSoftLimit)) { // 2000 is hardcoded end stop
                armSlideMotor.setPower(1); // extend continuously while button is held
                armSlideMotor.setDirection(DcMotor.Direction.REVERSE);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                armSlideDesiredPos = armSlideMotor.getCurrentPosition();
                armSlideLastMoveDirection = true; // forward
            } else if (gamepad1.dpad_down && (armSlideLastMoveDirection || Math.abs(armSlidePos) >= 50 )) { // encoder pos is inverted when in reverse; so it just checks to make sure it isn't within 35 of zero (due to belt slop)
                armSlideMotor.setPower(1);  // retract continuously while button is held
                armSlideMotor.setDirection(DcMotor.Direction.FORWARD);
                armSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Use builtin PID loop to run to position
                armSlideDesiredPos = armSlideMotor.getCurrentPosition(); // store current position in case the button isn't pressed next loop, so it knows where to hold
                armSlideLastMoveDirection = false; // backward
            } else {
                armSlideMotor.setTargetPosition(armSlideDesiredPos); // hold the motor at the position it was in last time it was moved
                armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION); // Use builtin PID loop to hold position
                armSlideMotor.setPower(armSlideHoldingPower); // Holding power
            }*/


            // Transport Mode
            /*if (gamepad1.y || gamepad2.y) {
                armPivotDesiredPos = setSignFromReference(700, armPivotDesiredPos);
                armSlideDesiredPos = setSignFromReference(200, armSlideDesiredPos);
            }*/

            // High basket preset
            /*if ((gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2) || gamepad2.right_trigger > 0.2) { // Both controller 1 triggers or controller 2 right trigger
                armPivotDesiredPos = setSignFromReference(2200, armPivotDesiredPos);
                armSlideDesiredPos = setSignFromReference(1650, armSlideDesiredPos);
            }*/

            // Outputs telemetry data to driver hub screen
            // telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
            // telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
            // telemetry.addData("Arm Slide Motor Power :", armSlideMotor.getPower());

//            telemetry.addData("Right Distance (mm): ", rightDistanceSensor.getDistance(DistanceUnit.MM));
//            telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
            telemetry.update();
        }
    }
}