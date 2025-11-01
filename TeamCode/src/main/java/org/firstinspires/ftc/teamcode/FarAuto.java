// Written primarily by Henry Rosenberg for AcaBots, FTC Team #24689
// hi also sze ting
// and Tsimur Havarko
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous
public class FarAuto extends LinearOpMode {
    DcMotor frontLeftMotor; // 1
    DcMotor backLeftMotor; // 0
    DcMotor frontRightMotor; // 1 (expansion)
    DcMotor backRightMotor; // 0 (expansion)
    CRServo intakeLeft; // 1
    CRServo intakeRight; // 0 (expansion)
    CRServo beltLeft; // 0
    CRServo beltRight; // 1 (expansion)
    CRServo beltVertical; // 2
    //    CRServo turret; // 2 (expansion)
    DcMotor flywheelRotateMotor; // 2 (expansion)
    DcMotor flywheelMotor; // 2
    DcMotor flywheelIntake; // 3
    Servo flywheelAngle; // 2 (expansion)
    // CRServo clawIntake;
    IMU imu;
    DistanceSensor rightDistanceSensor;
    DistanceSensor backDistanceSensor;
    double angle;


    private double calcLargestChange(double a, double b) {
        // Return the value of the greatest absolute value of either a or b. Used for dual controller input
        if (Math.abs(b) > Math.abs(a)) {
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

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        // Retrieve the IMU from the hardware map
        //imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot <------------------------------------------------------- IMPORTANT
        /*IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));*/
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        //imu.initialize(parameters);

        // Chassis-mounted distance sensors
//        rightDistanceSensor = hardwareMap.get(DistanceSensor.class, "rightDistanceSensor");
//        backDistanceSensor = hardwareMap.get(DistanceSensor.class, "backDistanceSensor");

        waitForStart();
        // 4. Drive forward for 1 second
        if (opModeIsActive()) {
            flywheelRotateMotor.setTargetPosition(0);
            flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flywheelRotateMotor.setPower(0.7);
            angle = 0.23;
            flywheelRotateMotor.setTargetPosition(200);
            flywheelRotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            flywheelRotateMotor.setPower(0.7);
            flywheelAngle.setPosition(angle);
            flywheelMotor.setPower(1);
            sleep(1500);
            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            beltLeft.setPower(1);
            beltRight.setPower(1);
            beltVertical.setPower(-1);
            flywheelIntake.setPower(0.5);
            sleep(2200);

            beltLeft.setPower(0);
            beltRight.setPower(0);
            beltVertical.setPower(0);
            flywheelIntake.setPower(0);
            sleep(2200);

            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            beltLeft.setPower(1);
            beltRight.setPower(1);
            beltVertical.setPower(-1);
            flywheelIntake.setPower(0.5);
            sleep(1000);

            intakeLeft.setPower(0);
            intakeRight.setPower(0);
            beltLeft.setPower(0);
            beltRight.setPower(0);
            beltVertical.setPower(0);
            flywheelIntake.setPower(0);
            sleep(2500);

            intakeLeft.setPower(1);
            intakeRight.setPower(1);
            beltLeft.setPower(1);
            beltRight.setPower(1);
            beltVertical.setPower(-1);
            flywheelIntake.setPower(0.5);
            sleep(2200);

            beltLeft.setPower(0);
            beltRight.setPower(0);
            beltVertical.setPower(0);
            flywheelIntake.setPower(0);




            frontRightMotor.setPower(0.4);
            frontLeftMotor.setPower(0.4);
            backRightMotor.setPower(0.4);
            backLeftMotor.setPower(0.4);

            sleep(1200);

            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);


            flywheelMotor.setPower(0);
            beltLeft.setPower(0);
            beltRight.setPower(0);
            beltVertical.setPower(0);
            flywheelIntake.setPower(0);


            frontRightMotor.setPower(0.3);
            backRightMotor.setPower(0.3);
            frontLeftMotor.setPower(-0.3);
            backLeftMotor.setPower(-0.3);
            sleep(1350);

            frontRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            backLeftMotor.setPower(0);

            flywheelRotateMotor.setTargetPosition(0);
            flywheelRotateMotor.setPower(0.7);
            sleep(1000);

        }

        //telemetry.addData("rmp flywheel: ", flywheelMotor.getPower());
        //telemetry.addData("rotShot", rotShot.getPower());
        //telemetry.update();


    }
}