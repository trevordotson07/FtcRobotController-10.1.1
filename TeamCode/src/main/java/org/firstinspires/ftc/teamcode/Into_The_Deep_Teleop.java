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

    //BNO055IMU imu;

    private ElapsedTime runtime = new ElapsedTime();
   /* public DcMotor front_left;
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
*/

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


        robot.init(hardwareMap);

        robot.slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

            //Lift System
            if (gamepad2.right_stick_y > 0.01) {
                robot.lift.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.01) {
                robot.lift.setPower(-0.5);
            } else {
                robot.lift.setPower(0);
            }

            if (gamepad2.left_stick_y > 0.01) {
                robot.slideRot.setPower(0.5);
            } else if (gamepad2.left_stick_y < -0.01) {
                robot.slideRot.setPower(-0.5);
            } else {
                robot.slideRot.setPower(0);
            }

            if (gamepad2.cross) {
                robot.slideSpool.setPower(0.85);
            } else if (gamepad2.triangle) {
                robot.slideSpool.setPower(-0.85);
            } else {
                robot.slideSpool.setPower(0);
            }

            if (gamepad1.dpad_down) {
                robot.collectionTilt.setPosition(0.6);
            } else if (gamepad1.dpad_up) {
                robot.collectionTilt.setPosition(0.835);
            }

            if (gamepad1.dpad_right) {
                robot.collectionPan.setPower(0.10);
            } else if (gamepad1.dpad_left) {
                robot.collectionPan.setPower(-0.10);
            } else {
                robot.collectionPan.setPower(0);

            }
            if (gamepad1.left_bumper) {
                robot.collectionIntake.setPosition(0.6);
            } else if (gamepad1.left_trigger > 0.01) {
                robot.collectionIntake.setPosition(0.7);
            }

            if (gamepad2.right_bumper) {
                robot.slideRot.setTargetPosition(750);
                robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideRot.setPower(0.95);
            } else if (gamepad2.right_trigger > 0.01) {
                robot.slideRot.setTargetPosition(950);
                robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideRot.setPower(0.95);
           }

            if (gamepad2.left_bumper) {
                robot.slideRot.setTargetPosition(robot.slideRot.getCurrentPosition());
                robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideRot.setPower(0.95);
            } else {
                robot.slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            if (gamepad2.left_trigger > 0.01) {
                robot.slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
            if (gamepad1.right_bumper) {
                robot.front_left.setPower(0.5 * Range.clip(forward + clockwise + right, -1, 1));
                robot.front_right.setPower(0.5 * Range.clip(forward - clockwise - right, -1, 1));
                robot.back_left.setPower(0.5 * Range.clip(forward + clockwise - right, -1, 1));
                robot.back_right.setPower(0.5 * Range.clip(forward - clockwise + right, -1, 1));
            } else if (gamepad1.right_trigger > 0.1) {
                robot.front_left.setPower(Range.clip(forward + clockwise + right, -1, 1));
                robot.front_right.setPower(Range.clip(forward - clockwise - right, -1, 1));
                robot.back_left.setPower(Range.clip(forward + clockwise - right, -1, 1));
                robot.back_right.setPower(Range.clip(forward - clockwise + right, -1, 1));
            } else {
                robot.front_left.setPower(0.75 * Range.clip(forward + clockwise + right, -1, 1));
                robot.front_right.setPower(0.75 * Range.clip(forward - clockwise - right, -1, 1));
                robot.back_left.setPower(0.75 * Range.clip(forward + clockwise - right, -1, 1));
                robot.back_right.setPower(0.75 * Range.clip(forward - clockwise + right, -1, 1));
            }
            telemetry.addData("SlideRot", robot.slideRot.getCurrentPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();
        }

    }
}

