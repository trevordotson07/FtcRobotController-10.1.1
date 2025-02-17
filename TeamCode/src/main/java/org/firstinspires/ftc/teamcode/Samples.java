package org.firstinspires.ftc.teamcode;
//import dependencies here (code auto does this)

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Samples", group = "14174")
public class Samples extends LinearOpMode {
    //defines variables for gyro
    double botHeading = 0;

    double offset = 0;
    Into_the_Deep_Hardware robot = new Into_the_Deep_Hardware();


    double headingResetValue;

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        robot.init(hardwareMap);

        robot.slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.slideSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.slideSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU imu;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        // Retrieve the IMU from the hardware map
        BNO055IMU.Parameters parameters2 = new BNO055IMU.Parameters();
        parameters2.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters2.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters2.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters2.loggingEnabled = true;
        parameters2.loggingTag = "IMU";
        parameters2.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Adjust the orientation parameters to match your robot
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters2);



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            double botHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + offset;

            if (gamepad1.right_bumper) {
                offset = -botHeading;
            }

            if (gamepad2.left_bumper) {
                robot.slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                robot.slideSpool.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            //Change motor speed based on button presses on controller 1
            if (gamepad1.right_bumper) {
                robot.front_left.setPower(0.75 * frontLeftPower);
                robot.front_right.setPower(0.75 * frontRightPower);
                robot.back_left.setPower(0.75 * backLeftPower);
                robot.back_right.setPower(0.75 * backRightPower);
            } else {
                robot.front_left.setPower(0.95 * frontLeftPower);
                robot.front_right.setPower(0.95 * frontRightPower);
                robot.back_left.setPower(0.95 * backLeftPower);
                robot.back_right.setPower(0.95 * backRightPower);
            }
            //Slide rotation
            if (gamepad2.left_stick_y > 0.01) {
                robot.slideRot.setPower(-0.5);
            } else if (gamepad2.left_stick_y < -0.01) {
                robot.slideRot.setPower(0.5);
            } else {
                robot.slideRot.setPower(0);
            }
            //Collection tilt
            if (gamepad1.cross) {
                robot.collectionTilt.setPosition(0.83);
            } else if (gamepad1.triangle) {
                robot.collectionTilt.setPosition(0.23);
            }
            //Collection Intake
            if (gamepad1.dpad_down) {
                robot.collectionIntake.setPower(0.53);
            } //else if (gamepad1.dpad_left) {
            //robot.collectionIntake.setPosition(0.72);}
            else if (gamepad1.dpad_up) {
                robot.collectionIntake.setPower(0.8);
            }
            //Resets slides encoder
            if (gamepad1.left_trigger > 0.01) {
                robot.slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.slideSpool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            if (gamepad2.right_bumper) {
                robot.slideRot.setTargetPosition(robot.slideRot.getCurrentPosition());
                robot.slideSpool.setTargetPosition(robot.slideSpool.getCurrentPosition());
                Lift(0.95);
            }
            if (!gamepad2.left_bumper) {
                SlidesControl(1350,0,-0.95, 0.95);
            }
            if (gamepad2.left_bumper) {
                SlidesControl(0,1350,0.95,-0.95);
                robot.collectionTilt.setPosition(0.8);
            }
            if (gamepad2.left_bumper && gamepad2.left_trigger > 0.01) {
                robot.collectionTilt.setPosition(0.5);
                robot.collectionIntake.setPower(0.5);
            }
            //Software Limit for the slides
            if (gamepad1.right_trigger < 0.01) {
                if (robot.slideSpool.getCurrentPosition() > 1350) {
                    if (gamepad2.cross) {
                        robot.slideSpool.setPower(0.85);
                    } else {
                        robot.slideSpool.setPower(0);
                    }
                } else {
                    if (gamepad2.cross) {
                        robot.slideSpool.setPower(0.85);
                    } else if (gamepad2.triangle) {
                        robot.slideSpool.setPower(-0.85);
                    } else {
                        robot.slideSpool.setPower(0);
                    }
                }
            } else {
                if (gamepad2.cross) {
                    robot.slideSpool.setPower(0.85);
                } else if (gamepad2.triangle) {
                    robot.slideSpool.setPower(-0.85);
                } else {
                    robot.slideSpool.setPower(0);
                }
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("heading", (Math.toDegrees(botHeading)));
            telemetry.addData("SlideRot", robot.slideRot.getCurrentPosition());
            telemetry.addData("Spool", robot.slideSpool.getCurrentPosition());
            telemetry.update();
        }

    }
    public void SlidesControl (int RotTarget, int SpoolTarget, double RotPower, double SpoolPower){
        robot.slideRot.setTargetPosition(RotTarget);
        robot.slideSpool.setTargetPosition(SpoolTarget);
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(RotPower);
        robot.slideSpool.setPower(SpoolPower);
    }
    public void Lift (double liftPower) {
        robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideSpool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.slideRot.setPower(liftPower);
        robot.slideSpool.setPower(liftPower);
    }
}

