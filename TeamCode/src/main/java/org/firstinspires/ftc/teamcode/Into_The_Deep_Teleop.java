package org.firstinspires.ftc.teamcode;
// import dependencies (code auto does this)
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

//creates teleop class for robot
@TeleOp(name="Into_the_Deep_Teleop", group="14174")
//@Disabled
public class Into_The_Deep_Teleop extends LinearOpMode {

    public static final String TAG = "Vuforia VuMark Sample";

    private static final String VUFORIA_KEY =
            "AQdreXP/////AAABmZt6Oecz+kEzpK0JGPmBsiNN7l/NAvoL0zpZPFQAslTHUcNYg++t82d9o6emZcSfRJM36o491JUmYS/5qdxxP235BssGslVIMSJCT7vNZ2iQW2pwj6Lxtw/oqvCLtgGRPxUyVSC1u5QHi+Siktg3e4g9rYzoQ2+kzv2chS8TnNooSoF6YgQh4FXqCYRizfbYkjVWtx/DtIigXy+TrXNn84yXbl66CnjNy2LFaOdBFrl315+A79dEYJ+Pl0b75dzncQcrt/aulSBllkA4f03FxeN3Ck1cx9twVFatjOCFxPok0OApMyo1kcARcPpemk1mqF2yf2zJORZxF0H+PcRkS2Sv92UpSEq/9v+dYpruj/Vr";
    // Declare OpMode members.

    Into_the_Deep_Hardware robot = new Into_the_Deep_Hardware();

    BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor lift;
    public DcMotor slideRot;
    public DcMotor slideSpool;

    //Define Servos
    public Servo collectionTilt;
    public CRServo collectionPan;
    public Servo collectionIntake;


    //@Override
    public void runOpMode() //throws InterruptedException
    {

        BNO055IMU imu;
        // Retrieve the IMU from the hardware map, gyro in the control hub
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters2);


        //Drive motors
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");

        //Lift System
        lift = hardwareMap.get(DcMotor.class, "lift");
        slideRot = hardwareMap.get(DcMotor.class, "slideRot");
        slideSpool = hardwareMap.get(DcMotor.class, "slideSpool");

        //Collection System
        collectionTilt = hardwareMap.get(Servo.class, "collectionTilt");
        collectionPan = hardwareMap.get(CRServo.class, "collectionPan");
        collectionIntake = hardwareMap.get(Servo.class, "collectionIntake");


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Lift System
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets motor direction for mecanum wheels
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);
        //Servo Speed


        robot.init(hardwareMap);

        //Variables

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double clockwise = gamepad1.right_stick_x;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //waitForStart();
        runtime.reset();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Status:", "Waiting for start command.");
            telemetry.update();

        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------
            // DRIVING CONTROLS
            //-----------------------------------------------------------------------------------------
            //-----------------------------------------------------------------------------------------


            if (Math.abs(gamepad1.left_stick_y) > 0.01) {
                if (gamepad1.left_stick_y > 0) {
                    forward = -(gamepad1.left_stick_y * gamepad1.left_stick_y);
                } else if (gamepad1.left_stick_y < 0) {
                    forward = (gamepad1.left_stick_y * gamepad1.left_stick_y);
                }
            } else {
                forward = 0;
            }

            if (Math.abs(gamepad1.left_stick_x) > 0.01) {
                if (gamepad1.left_stick_x > 0) {
                    right = gamepad1.left_stick_x * gamepad1.left_stick_x;
                } else if (gamepad1.left_stick_x < 0) {
                    right = -(gamepad1.left_stick_x * gamepad1.left_stick_x);
                }
            } else {
                right = 0;
            }

            if (Math.abs(gamepad1.right_stick_x) > 0.01) {
                if (gamepad1.right_stick_x > 0) {
                    clockwise = gamepad1.right_stick_x * gamepad1.right_stick_x;
                } else if (gamepad1.right_stick_x < 0) {
                    clockwise = -(gamepad1.right_stick_x * gamepad1.right_stick_x);
                }
            } else {
                clockwise = 0;
            }

            if (gamepad1.right_bumper) {
                right = -right;
            }

            //Lift System
            if (gamepad2.right_stick_y > 0.01) {
                lift.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.01) {
                lift.setPower(-0.5);
            } else {
                lift.setPower(0);
            }

            if (gamepad2.left_stick_y > 0.01) {
                slideRot.setPower(0.5);
            } else if (gamepad2.left_stick_y < -0.01) {
                slideRot.setPower(-0.5);
            } else {
                slideRot.setPower(0);
            }

            if (gamepad2.cross) {
                slideSpool.setPower(0.85);
            } else if (gamepad2.triangle) {
                slideSpool.setPower(-0.85);
            } else {
                slideSpool.setPower(0);
            }

            if (gamepad2.touchpad_finger_1_x > 0.01) {
                collectionPan.setPower(0.15);
            } else if (gamepad2.touchpad_finger_1_x < -0.01) {
                collectionPan.setPower(-0.15);
            } else {
                collectionPan.setPower(0);
            }

            if (gamepad2.touchpad_finger_1_y > 0.01) {
                collectionTilt.setPosition(collectionTilt.getPosition() + 0.05);
            } else if (gamepad2.touchpad_finger_1_y < -0.01) {
                collectionTilt.setPosition(collectionTilt.getPosition() - 0.05);
            }

            if (gamepad1.dpad_down) {
                collectionIntake.setPosition(0.5);
            } else if (gamepad1.dpad_up) {
                collectionIntake.setPosition(0);
            }

            /*
            if((forward > 0) &! (gamepad1.left_trigger > 0.1)) {
                clockwise = Range.clip(clockwise + (-0.035 * forward) + (0.035 * right), -1, 1 );
            }
            if((forward < 0) &! (gamepad1.left_trigger > 0.1)) {
                clockwise = Range.clip(clockwise + (-0.025 * forward) + (0.035 * right), -1, 1 );
            }
            */
            //set moter power based on button pressed and calculates the robot movement from the joystick values
            if (gamepad1.right_trigger > 0.1 ) {
                front_left.setPower(0.5 * Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(0.5 * Range.clip(forward - clockwise + right, -1, 1));
                back_left.setPower(0.5 * Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(0.5 * Range.clip(forward - clockwise - right, -1, 1));
            }else if (gamepad1.left_trigger > 0.1) {
                front_left.setPower(Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(Range.clip(forward - clockwise + right, -1, 1));
                back_left.setPower(Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(Range.clip(forward - clockwise - right, -1, 1));
            }
            else  {
                front_left.setPower(0.75 * Range.clip(forward + clockwise + right, -1, 1));
                front_right.setPower(0.75 * Range.clip(forward - clockwise + right, -1, 1));
                back_left.setPower(0.75 * Range.clip(forward + clockwise - right, -1, 1));
                back_right.setPower(0.75 * Range.clip(forward - clockwise - right, -1, 1));
            }



            //ServoTest


            telemetry.addData("GamePad2Pos", gamepad2.left_stick_y);
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }
}

