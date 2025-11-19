// Written by AcaBots FTC team 24689 for the 2025-26 DECODE Season

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

@Autonomous
public class odo9BallFarBlue extends LinearOpMode {
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
    GoBildaPinpointDriver odo;
    double oldTime = 0;

    public void StopAll() {
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
        beltLeft.setPower(0);
        beltRight.setPower(0);
        beltVertical.setPower(0);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
        flywheelMotor.setPower(0);
        flywheelIntake.setPower(0);
        flywheelRotateMotor.setPower(0);
    }

    public void shootThree()
    {
        // Intake balls into turret to shoot all 3
        flywheelIntake.setPower(0.5);

        sleep(3800); // Wait for all 3 to shoot

        flywheelIntake.setPower(0); // stop shooting but leave intake and flywheel running
    }

    @Override
    public void runOpMode() throws InterruptedException {

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
        flywheelMotor = hardwareMap.dcMotor.get("flywheelMotor");
        flywheelIntake = hardwareMap.dcMotor.get("flywheelIntake");
        flywheelAngle = hardwareMap.get(Servo.class, "flywheelAngle");

        // Reverse some of the drive motors depending on physical setup
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Setup odometry params
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        odo.setOffsets(0, 0, DistanceUnit.INCH); // <-------------------------------------------- NO IDEA WHAT THIS DOES AT ALL
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Initialize odometry stuff when match starts
        odo.resetPosAndIMU();

        // For RoadRunner pathing
        odo.setHeading(180, AngleUnit.DEGREES); // Set initial angle
        Pose2d startPose = new Pose2d(59, -12, Math.toRadians(180)); // starting coordinates and heading
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            odo.update();
            // Get status of the odometry (error stuff)
            telemetry.addData("Odometry Status: ", odo.getDeviceStatus());
            telemetry.update();

            // Start up all belts
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            beltLeft.setPower(1);
            beltRight.setPower(1);
            beltVertical.setPower(-1);

            // Shoot preloaded 3
            flywheelRotateMotor.setTargetPosition(0);
            flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flywheelRotateMotor.setPower(0.7);
            flywheelRotateMotor.setTargetPosition(217);
            flywheelAngle.setPosition(0.23); // Shooting angle for next far location
            flywheelMotor.setPower(0.92); // For far location

            sleep(2200); // warm up flywheel

            shootThree(); // from far location

            flywheelRotateMotor.setTargetPosition(0); // Return turret to center for next shots

            // Pickup first row of artifacts
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .splineTo(new Vector2d(36, -20), Math.toRadians(270)) // curve toward close artifact row
                            .lineToYConstantHeading(-58) // drive forward to intake artifacts
                            .build());

            flywheelRotateMotor.setTargetPosition(-450); // set turret to 45deg for next shots
            flywheelMotor.setPower(0.76); // For middle location
            flywheelAngle.setPosition(0.2); // Shooting angle for middle
            sleep(200); // chill for ball to be sucked in

            // reverse out and go to center
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(-24)
                            .setReversed(true) // make the spline more sensible
                            .splineToConstantHeading(new Vector2d(-16, -12), Math.toRadians(270)) // go to halfway
                            .splineToConstantHeading(new Vector2d(-12, -6), Math.toRadians(270)) // go to center-ish and point at goal
                            .build());

            shootThree();

            // Pickup middle row of artifacts
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .setReversed(false) // make the spline more sensible
                            .splineToConstantHeading(new Vector2d(12, -20), Math.toRadians(270)) // Face row and drive to front of it
                            .lineToYConstantHeading(-58) // drive forward to intake artifacts
                            .build());

            sleep(200); // chill for ball to be sucked in

            // reverse out and go to center
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(-24)
                            .setReversed(true) // make the spline more sensible
                            .splineToConstantHeading(new Vector2d(-12, -6), Math.toRadians(270)) // go to center-ish and point at goal
                            .build());

            shootThree();

            flywheelRotateMotor.setTargetPosition(0); // Return turret to center

            // Get out of launch zone
            Actions.runBlocking(
                    drive.actionBuilder(drive.localizer.getPose())
                            .lineToYConstantHeading(-53) // get out of launch zone and pickup next row
                            .build());
            StopAll();
        }
    }
}