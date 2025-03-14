package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_B;



import android.app.Activity;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

public class ClawCode_B{
    Rev2mDistanceSensor distanceSensor;
    private String currentState;
    Servo servo;
    NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];
    public ClawCode_B(HardwareMap hardwareMap) {
        currentState = "IDLE";
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        servo = hardwareMap.get(Servo.class, "IS");
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "DS");
    }

    public void update(){
        switch (currentState){
            case "IDLE":
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                if (hsvValues[0]>200){
                    servo.setPosition(0);
                }else {servo.setPosition(1);}
                break;
            case "GRABBED":
                if (distanceSensor.getDistance(DistanceUnit.CM)<20 ){servo.setPosition(1);

                }



                break;

            default:
                break;
        }
    }
    // todo: write your code here
}