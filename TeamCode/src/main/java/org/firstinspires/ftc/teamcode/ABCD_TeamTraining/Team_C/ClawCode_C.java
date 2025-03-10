package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_C;
import android.app.Activity;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.Telemetry;
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



public class ClawCode_C {
    NormalizedColorSensor colorSensor;
    Servo   servo;
    DistanceSensor sensorDistance;

    final float[] hsvValues = new float[3];






    private String currentState;

    public ClawCode_C(HardwareMap hardwareMap, Telemetry telemetry){
        servo = hardwareMap.get(Servo.class, "left_hand");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;

        currentState = "Idle";
    }

    // todo: write your code here
    public void update(){
        switch(currentState){
            case "Idle":
                Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorDistance;
                NormalizedRGBA colors = colorSensor.getNormalizedColors();
                Color.colorToHSV(colors.toColor(), hsvValues);
                sensorDistance.getDistance(DistanceUnit.CM);
                if(hsvValues[0]>200 && sensorDistance.getDistance(DistanceUnit.CM)>10){
                    servo.setPosition(0);
                    currentState = "Grabbed";

                }
                break;
            case "Grabbed":
                if (sensorDistance.getDistance(DistanceUnit.CM)<10){
                    servo.setPosition(1);
                    currentState = "Idle";
                }
                break;
            default:

                break;
        }
    }
}
