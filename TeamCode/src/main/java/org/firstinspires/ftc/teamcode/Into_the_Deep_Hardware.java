package org.firstinspires.ftc.teamcode;
//import dependencies here (code auto does this)
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Into_the_Deep_Hardware
{
    /* Public OpMode members. */
    //DEFINE MOTORS
    public DcMotor front_left;
    public DcMotor front_right;
    public DcMotor back_left;
    public DcMotor back_right;

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



    }
}
