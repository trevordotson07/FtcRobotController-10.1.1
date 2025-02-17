package org.firstinspires.ftc.teamcode;
//import dependencies here (code auto does this)

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Into_the_Deep_Hardware
{
    /* Public OpMode members. */
    //DEFINE MOTORS
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;
    public DcMotor slideRot;
    public DcMotor slideSpool;

    //Define Servos
    public Servo collectionTilt;
    //public Servo collectionPan;
    public CRServo collectionIntake;

    /* local OpMode members. */
    //calls hardware map
    HardwareMap hwMap =  null;
    //creates a timer
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Into_the_Deep_Hardware(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        //Drive motors
        front_left = hwMap.get(DcMotor.class, "front_left");
        front_right = hwMap.get(DcMotor.class, "front_right");
        back_left = hwMap.get(DcMotor.class, "back_left");
        back_right = hwMap.get(DcMotor.class, "back_right");

        //Lift System
        slideRot = hwMap.get(DcMotor.class, "slideRot");
        slideSpool = hwMap.get(DcMotor.class, "slideSpool");

        //Collection System
        collectionTilt = hwMap.get(Servo.class, "collectionTilt");
        //collectionPan = hwMap.get(Servo.class, "collectionPan");
        collectionIntake = hwMap.get(CRServo.class, "collectionIntake");


        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Lift System

        slideRot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideSpool.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //sets motor direction for mecanum wheels
        front_right.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.FORWARD);

        //Sets motor direction for other motors
        slideSpool.setDirection(DcMotorSimple.Direction.REVERSE);
        slideRot.setDirection(DcMotorSimple.Direction.FORWARD);

//      slideRot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      slideRot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      slideRot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
