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

@TeleOp(name = "Speciman", group = "14174")
public class Speciman extends LinearOpMode {
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

        robot.slideSpool.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
                robot.front_left.setPower(0.5 * frontLeftPower);
                robot.front_right.setPower(0.5 * frontRightPower);
                robot.back_left.setPower(0.5 * backLeftPower);
                robot.back_right.setPower(0.5 * backRightPower);
            } else if (gamepad1.right_trigger > 0.1) {
                robot.front_left.setPower(frontLeftPower);
                robot.front_right.setPower(frontRightPower);
                robot.back_left.setPower(backLeftPower);
                robot.back_right.setPower(backRightPower);
            } else {
                robot.front_left.setPower(0.75 * frontLeftPower);
                robot.front_right.setPower(0.75 * frontRightPower);
                robot.back_left.setPower(0.75 * backLeftPower);
                robot.back_right.setPower(0.75 * backRightPower);
            }

            //Lift System
            if (gamepad2.right_stick_y > 0.01) {
                robot.lift.setPower(0.5);
            } else if (gamepad2.right_stick_y < -0.01) {
                robot.lift.setPower(-0.5);
            } else {
                robot.lift.setPower(0);
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
            if (gamepad1.dpad_down) {
                robot.collectionTilt.setPosition(0.51);
            } else if (gamepad1.dpad_up) {
                robot.collectionTilt.setPosition(0.83);
            }
            //Collection Pan
            if (gamepad1.dpad_right) {
                robot.collectionPan.setPower(0.40);
            } else if (gamepad1.dpad_left) {
                robot.collectionPan.setPower(-0.40);
            } else {
                robot.collectionPan.setPower(0);
            }
            //Collection Intake
            if (gamepad1.left_bumper) {
                robot.collectionIntake.setPosition(0.6);
            } else if (gamepad1.left_trigger > 0.01) {
                robot.collectionIntake.setPosition(0.7);
            }
            // Sets the slides to a vertical position
            if (gamepad2.right_bumper) {
                robot.slideRot.setTargetPosition(1350);
                robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideRot.setPower(0.95);
            }
            // Holds the slides to the current position
            if (gamepad2.left_bumper) {
                robot.slideRot.setTargetPosition(robot.slideRot.getCurrentPosition());
                robot.slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.slideRot.setPower(0.95);
            } else {
                robot.slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            //Resets slides encoder
            if (gamepad2.left_trigger > 0.01) {
                robot.slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //Software Limit for the slides
            if (gamepad2.right_trigger < 0.01) {
                if (robot.slideSpool.getCurrentPosition() > 1350) {
                    if (gamepad2.cross) {
                        robot.slideSpool.setPower(-0.85);
                    } else {
                        robot.slideSpool.setPower(0);
                    }
                } else {
                    if (gamepad2.cross) {
                        robot.slideSpool.setPower(-0.85);
                    } else if (gamepad2.triangle) {
                        robot.slideSpool.setPower(0.85);
                    } else {
                        robot.slideSpool.setPower(0);
                    }
                }
            } else {
                if (gamepad2.cross) {
                    robot.slideSpool.setPower(-0.85);
                } else if (gamepad2.triangle) {
                    robot.slideSpool.setPower(0.85);
                } else {
                    robot.slideSpool.setPower(0);
                }
            }


            telemetry.addData("Status", "Running");
            telemetry.addData("heading", (Math.toDegrees(botHeading)));
            telemetry.update();
        }
    }
}


