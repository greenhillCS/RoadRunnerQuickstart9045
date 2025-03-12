package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_D;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import android.app.Activity;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;



public class ClawCode_D {


    private DistanceSensor sensorDistance;

    Servo servo;
    NormalizedColorSensor colorSensor;
    final float[] hsvValues = new float[3];


    private String currentState;


    public ClawCode_D(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = hardwareMap.get(Servo.class, "CLAW");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "CS");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "DS");
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        currentState = "Open";
        Color.colorToHSV(colors.toColor(), hsvValues);


    }

    public void update() {
        switch (currentState) {
            case "Open":
                //define var
                //loop to check conditions
                if (hsvValues[0] > 200 && sensorDistance.getDistance(DistanceUnit.CM) < 10) {
                    servo.setPosition(1);
                    currentState = "FULLTOBURSTING";
                    //tell it to do something and switch to other case
                }
                break;

            case "FULLTOBURSTING":
                if (sensorDistance.getDistance(DistanceUnit.CM) > 20) {
                    servo.setPosition(0);
                    currentState = "Open";
                }
                break;

            default:
                break;

        }

    }
}

