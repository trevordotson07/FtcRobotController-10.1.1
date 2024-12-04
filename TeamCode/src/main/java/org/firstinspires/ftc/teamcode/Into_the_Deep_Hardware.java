package org.firstinspires.ftc.teamcode;
//import dependencies here (code auto does this)
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public DcMotor lift;
    public DcMotor slideRot;
    public DcMotor slideSpool;
    public DcMotor slideRetract;

    //Define Servos
    public Servo collectionTilt;
    public Servo collectionPan;
    public Servo collectionIntake;

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

        front_left = hwMap.get(DcMotor.class, "front_left");
        front_right = hwMap.get(DcMotor.class, "front_right");
        back_left = hwMap.get(DcMotor.class, "back_left");
        back_right = hwMap.get(DcMotor.class, "back_right");

        //Lift System
        lift = hwMap.get(DcMotor.class, "lift");
        slideRot = hwMap.get(DcMotor.class, "slideRot");
        slideSpool = hwMap.get(DcMotor.class, "slideSpool");
        slideRetract = hwMap.get(DcMotor.class, "slideRetract");

        //Collection System
        collectionTilt = hwMap.get(Servo.class, "collectionTilt");
        collectionPan = hwMap.get(Servo.class, "collectionPan");
        collectionIntake = hwMap.get(Servo.class, "collectionIntake");
    }
}
