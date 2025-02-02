package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;


@Disabled
@TeleOp(name = "Auto_Record", group = "14174")
public class Auto_Record extends LinearOpMode {
    //List of each "Frame" of the recording | Each frame has multiple saved values that are needed to fully visualize it
    ArrayList<HashMap<String, Double>> recording = new ArrayList<>();


    final ElapsedTime runtime = new ElapsedTime();
    boolean isPlaying = false;
    int frameCounter = 0;
    int robotState = 0;


    //Variables for hardware map
    Into_the_Deep_Hardware robot = new Into_the_Deep_Hardware();


    @Override
    public void runOpMode() {
        //Attaching the variables declared with the physical motors by name or id
        robot.init(hardwareMap);

        //Increasing efficiency in getting data from the robot
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addData("Status", "Waiting to Start");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {


                //Before recording, gives driver a moment to get ready to record
                //Once the start button is pressed, recording will start
                if (gamepad1.start && robotState == 0) {
                    robotState = 1;
                    runtime.reset();
                    telemetry.addData("Status", "Recording");
                    telemetry.addData("Time until recording end", 15 - runtime.time() + "");
                }
                else if(robotState == 0){
                    telemetry.addData("Status", "Waiting to start recording");
                    telemetry.addData("Version", "1");
                }


                //The recording has started and inputs from the gamepad are being saved in a list
                else if(robotState == 1){
                    if(15 - runtime.time() > 0){
                        telemetry.addData("Status", "Recording");
                        telemetry.addData("Time until recording end", 15 - runtime.time() + "");
                        HashMap<String, Double> values = robotMovement();
                        recording.add(values);
                    }else{
                        robotState = 2;
                    }
                }

                //PAUSE BEFORE REPLAYING RECORDING
                //Reset the robot position and samples

                //Press START to play the recording
                else if(robotState == 2){
                    telemetry.addData("Status", "Waiting to play Recording" + recording.size());
                    telemetry.addData("Time", runtime.time() + "");
                    if (gamepad1.start){
                        runtime.reset();
                        robotState = 3;
                        telemetry.addData("Status", "Playing Recording");
                        telemetry.update();
                        isPlaying = true;
                        playRecording(recording);
                    }
                }

                //Play-back the recording(This is very accurate to what the autonomous will accomplish)
                //WARNING: I recommend replaying the recording(What was driven and what was replayed vary a lot!)

                //Press the X button to stop(The recording does not stop on its own)
                else if(robotState == 3){
                    if(gamepad1.x){
                        isPlaying = false;
                    }
                    if(isPlaying){
                        playRecording(recording);
                    }else{
                        robotState = 4;
                        telemetry.addData("Status", "Done Recording play-back");
                        telemetry.addData("Save to file", "Press start to save");
                        telemetry.update();
                    }
                }

                //Press START one last time to save the recording
                //After you see the confirmation, you may stop the program.
                else if(robotState == 4){
                    if(gamepad1.start){
                        telemetry.addData("Status", "Saving File");
                        boolean recordingIsSaved = false;
                        String path = String.format("%s/FIRST/data/Auto_Left_Collection.fil",
                                Environment.getExternalStorageDirectory().getAbsolutePath());



                        telemetry.clearAll();
                        telemetry.addData("Status", saveRecording(recording, path));
                        telemetry.update();


                    }
                }



                telemetry.update();
            }


        }
    }

    //Writes the recording to file
    public String saveRecording(ArrayList<HashMap<String, Double>> recording, String path){
        String rv = "Save Complete";

        try {
            File file = new File(path);

            FileOutputStream fos = new FileOutputStream(file);
            ObjectOutputStream oos = new ObjectOutputStream(fos);

            oos.writeObject(recording);
            oos.close();
        }
        catch(IOException e){
            rv = e.toString();
        }

        return rv;
    }




    //Think of each frame as a collection of every input the driver makes in one moment, saved like a frame in a video is
    private void playRecording(ArrayList<HashMap<String, Double>> recording){
        //Gets the correct from from the recording

        //The connection between the robot and the hub is not very consistent, so I just get the inputs from the closest timestamp
        //and use that
        double largestTime = 0;
        int largestNum = 0;
        int correctTimeStamp = 0;
        for(int i = 0; i < recording.size();i++){
            if(recording.get(i).get("time") > largestTime){
                if(recording.get(i).get("time") <= runtime.time()){
                    largestTime = recording.get(i).get("time");
                    largestNum = i;
                }
                else{
                    correctTimeStamp = largestNum;
                }
            }
        }
        //Only used inputs are saved to the final recording, the file is too large if every single timestamp is saved.
        telemetry.addData("correctTimeStamp", correctTimeStamp + "");
        telemetry.update();
        HashMap<String, Double> values = recording.get(correctTimeStamp);


        double forward = values.getOrDefault("rotY", 0.0);
        double right = values.getOrDefault("rotX", 0.0);
        double turn = values.getOrDefault("rx", 0.0);


        double highestValue = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);




        //Calculates amount of power for each wheel to get the desired outcome
        //E.G. You pressed the left joystick forward and right, and the right joystick right, you strafe diagonally while at the same time turning right, creating a circular strafing motion.
        //E.G. You pressed the left joystick forward, and the right joystick left, you drive like a car and turn left
        if(highestValue >= 0.1){
            robot.front_left.setPower((forward + right + turn) / highestValue);
            robot.front_right.setPower((forward - right - turn) / highestValue);
            robot.back_left.setPower((forward + right - turn) / highestValue);
            robot.back_right.setPower((forward - right + turn) / highestValue);
        }
    }




    //Simple robot movement
    //Slowed to half speed so movements are more accurate
    private HashMap<String, Double> robotMovement() {
        frameCounter++;
        HashMap<String, Double> values = new HashMap<>();
        double highestValue;

        double forward = -gamepad1.left_stick_y; //Controls moving forward/backward
        double right = gamepad1.left_stick_x; //Controls strafing left/right       *the 1.1 multiplier is to counteract any imperfections during the strafing*
        double turn = gamepad1.right_stick_x; //Controls turning left/right
        forward /= 2;
        right /= 2;
        turn /= 2;


        values.put("rotY", forward);
        values.put("rotX", right);
        values.put("rx", turn);
        values.put("time", runtime.time());

        //Makes sure power of each engine is not below 100% (Math cuts anything above 1.0 to 1.0, meaning you can lose values unless you change values)
        //This gets the highest possible outcome, and if it's over 1.0, it will lower all motor powers by the same ratio to make sure powers stay equal
        highestValue = Math.max(Math.abs(forward) + Math.abs(right) + Math.abs(turn), 1);

        //Calculates amount of power for each wheel to get the desired outcome
        //E.G. You pressed the left joystick forward and right, and the right joystick right, you strafe diagonally while at the same time turning right, creating a circular strafing motion.
        //E.G. You pressed the left joystick forward, and the right joystick left, you drive like a car and turn left
        if(highestValue >= 0.3){
            robot.front_left.setPower((forward + right + turn) / highestValue);
            robot.front_right.setPower((forward - right + turn) / highestValue);
            robot.back_left.setPower((forward - right - turn) / highestValue);
            robot.back_right.setPower((forward + right - turn) / highestValue);
        }


        return values;
    }






}