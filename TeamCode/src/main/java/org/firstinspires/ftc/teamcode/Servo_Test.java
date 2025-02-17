package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Servo_Test", group="14174")
//@Disabled
public class Servo_Test extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    Servo Servo1;
    Servo Servo2;
    CRServo Servo3;
    Servo Servo4;

    DcMotor Motor1;
    double Motor1Power = 0;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        Servo1 = hardwareMap.servo.get("collectionPan");
        Servo2 = hardwareMap.servo.get("collectionTilt");
        Servo3 = hardwareMap.crservo.get("collectionIntake");
        //Servo4 = hardwareMap.servo.get("NAME");

        //Servo1.setPosition(0.7);
        //Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        Servo2.setDirection(Servo.Direction.FORWARD);
        Servo1.setDirection(Servo.Direction.FORWARD);


        idle();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            //gamepad 1 for servos gamepad 2 for motors


            if (gamepad1.a) {
                Servo1.setPosition(Servo1.getPosition() + .001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }
            if (gamepad1.b) {
                Servo1.setPosition(Servo1.getPosition() - .001);
                telemetry.addData("Servo1 Position", Servo1.getPosition());
                telemetry.addData("Direction", Servo1.getDirection());
                telemetry.update();
            }
            if(gamepad1.x) {
                Servo2.setPosition(Servo2.getPosition() + .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();
            }
            if(gamepad1.y) {
                Servo2.setPosition(Servo2.getPosition() - .001);
                telemetry.addData("Servo2 Position", Servo2.getPosition());
                telemetry.addData("Direction", Servo2.getDirection());
                telemetry.update();

            }

            }


        }
            }



