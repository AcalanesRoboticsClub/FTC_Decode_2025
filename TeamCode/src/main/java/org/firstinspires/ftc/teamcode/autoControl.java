// Written primarily by Henry Rosenberg for AcaBots, FTC Team #24689

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class autoControl extends LinearOpMode {
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    DcMotor armPivotMotor;
    DcMotor armSlideMotor;
    Servo rightHang;
    Servo leftHang;
    Servo clawWrist;
    CRServo clawIntake;
    IMU imu;
    DistanceSensor rightDistanceSensor;
    DistanceSensor backDistanceSensor;

    private void omniMoveByTimeDirection(double xPower, double yPower, double rxPower, boolean runForTime, double desiredRuntimeSeconds) {
        telemetry.addData("Auto Status :", "Inside chassis move function \n");
        telemetry.update();

        double max;

        // Omni movement equations
        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower  = yPower + xPower + rxPower;
        double frontRightPower = yPower - xPower - rxPower;
        double backLeftPower   = yPower - xPower + rxPower;
        double backRightPower  = yPower + xPower - rxPower;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }


        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send calculated power to wheels
        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);

        telemetry.addData("FL Power: ", frontLeftPower);
        telemetry.addData("FR Power: ", frontRightPower);
        telemetry.addData("BL Power: ", backLeftPower);
        telemetry.addData("BR Power: ", backRightPower);
        telemetry.update();

        // if in run for time mode, otherwise waiting will be handled externally
        if (runForTime) {
            double startTime = getRuntime(); // in seconds
            // Wait for movement to finish
            while ((getRuntime() - startTime) < desiredRuntimeSeconds && opModeIsActive()) {
                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Moving chassis \n");
                telemetry.addData("Elapsed Time :", (getRuntime() - startTime));
                telemetry.addData("Desired Runtime :", desiredRuntimeSeconds);
                telemetry.update();
            }

            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);
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
        armPivotMotor = hardwareMap.dcMotor.get("armPivotMotor");
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks


        // Arm Slide Motor
        // Encoder of -2130 is fully extended
        armSlideMotor = hardwareMap.dcMotor.get("armSlideMotor");
        armSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset the motor encoder so that it reads zero ticks

        // Hanging Claws
        rightHang = hardwareMap.get(Servo.class, "rightHangServo");
        leftHang = hardwareMap.get(Servo.class, "leftHangServo");

        // Game Element Intake Claw
        clawWrist = hardwareMap.get(Servo.class, "clawWristServo");
        clawIntake = hardwareMap.get(CRServo.class, "clawIntakeServo");
        clawWrist.setPosition(-1); // start within the starting config


        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Chassis-mounted distance sensors
        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        if (opModeIsActive()) { // Used to be a while loop
            telemetry.addData("Auto Status :", "Started \n");
            telemetry.update();

            // Hold specimen lightly
            clawIntake.setPower(-0.0528);


            // Pivot arm upward
            armPivotMotor.setTargetPosition(1620);
            armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armPivotMotor.setPower(1);

            // Slide the slide out
            armSlideMotor.setTargetPosition(-1100);
            armSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armSlideMotor.setPower(0.5);

            // wait for the arm to finish raising and extending
            while (armPivotMotor.getCurrentPosition() < 1550 || armSlideMotor.getCurrentPosition() > -1060) {
                if (isStopRequested()) {
                    armPivotMotor.setPower(0);
                    armSlideMotor.setPower(0);
                    return;
                }

                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for arm & alide movement to finish \n");
                telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
                telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
                telemetry.update();
            }

            sleep(1000);


            telemetry.addData("Auto Status : ", "Moving forward to submersible");

            // Move forward
            omniMoveByTimeDirection(0.0, 0.3, 0, false, 0);

            while(backDistanceSensor.getDistance(DistanceUnit.MM) < 620 && opModeIsActive()) {
                telemetry.addData("Auto Status : ", "Moving forward using distance sensor");
                telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // Turn off the motors after movement is finished
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            sleep(500); // let everything settle

            // Lower arm to contact rung
            armPivotMotor.setTargetPosition(1230);
            armPivotMotor.setPower(1);


            // Wait for the arm to finish lowering
            while (armPivotMotor.getCurrentPosition() > 1280) {
                if (isStopRequested()) {
                    armPivotMotor.setPower(0);
                    return;
                }
                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for arm pivot to lower \n");
                telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
                telemetry.update();
            }


            // Pull the arm slide back in
            armSlideMotor.setTargetPosition(-1000);
            armSlideMotor.setPower(1);

            // wait for the specimen to be attached, wait until the arm is finish retracting and it has been trying for 2 seconds
            double preEjectRuntime = getRuntime();
            while (armSlideMotor.getCurrentPosition() < - 1050 && ((getRuntime() - preEjectRuntime) < 2.0)) {
                if (isStopRequested()) {
                    armSlideMotor.setPower(0);
                    return;
                }

                // Outputs telemetry data to driver hub screen
                telemetry.clearAll();
                telemetry.addData("Auto Status :", "Waiting for specimen to eject \n");
                telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
                telemetry.update();
            }


            telemetry.addData("Auto Status : ", "Moving back away from submersible");


            // Move back to park for a given time and power
            omniMoveByTimeDirection(0.0, -0.3, 0, false, 0);

            while(backDistanceSensor.getDistance(DistanceUnit.MM) > 400 && opModeIsActive()) {
                telemetry.addData("Auto Status : ", "Moving back using distance sensor");
                telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // Turn off the motors after movement is finished
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            sleep(1500);

            // Turn off intake after releasing
            clawIntake.setPower(0);


            // Set everything back to fitting inside the robot frame
            armSlideMotor.setTargetPosition(0);
            armSlideMotor.setPower(0.5);

            // wait for slide to retract
            sleep(800);

            armPivotMotor.setTargetPosition(50);
            armPivotMotor.setPower(1);

            sleep(2000);

            // High power not needed to just hold retracted
            armPivotMotor.setPower(0.25);
            armSlideMotor.setPower(0.25);

            telemetry.addData("Auto Status :", "Starting movement to park \n");
            telemetry.update();

            // Move right to park. When runForTime is false, it only sets the motor powers, and they need to be turned off later. last parameter is ignored. rx is counteracting weirdness
            omniMoveByTimeDirection(0.3, 0.0, 0.02, false, 0);

            while(rightDistanceSensor.getDistance(DistanceUnit.MM) > 200 && opModeIsActive()) {
                telemetry.addData("Auto Status : ", "Moving right using distance sensor");
                telemetry.addData("Right Distance (mm): ", rightDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // Turn off the motors after movement is finished
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            telemetry.addData("Auto Status : ", "Moving back to park");

            sleep(200); // let everything settle

            // Move back to park for a given time and power
            omniMoveByTimeDirection(0.0, -0.3, 0, false, 0);

            while(backDistanceSensor.getDistance(DistanceUnit.MM) > 150 && opModeIsActive()) {
                telemetry.addData("Auto Status : ", "Moving right using distance sensor");
                telemetry.addData("Back Distance (mm): ", backDistanceSensor.getDistance(DistanceUnit.MM));
                telemetry.update();
            }

            // Turn off the motors after movement is finished
            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            sleep(500); // let everything settle

            // Outputs telemetry data to driver hub screen
            telemetry.clearAll();
            telemetry.addData("Auto Status :", "Finished! \n");
            telemetry.addData("Arm Pivot Encoder Position :", armPivotMotor.getCurrentPosition());
            telemetry.addData("Arm Slide Encoder Position :", armSlideMotor.getCurrentPosition());
            telemetry.addData("Right Hang Servo Position :", rightHang.getPosition());
            telemetry.addData("Left Hang Servo Position :", leftHang.getPosition());
            telemetry.addData("Claw Wrist Servo Position :", clawWrist.getPosition());
            telemetry.addData("Claw Intake Servo Power :", clawIntake.getPower());
            telemetry.update();

            armPivotMotor.setPower(0);
            armSlideMotor.setPower(0);
        }
    }
}