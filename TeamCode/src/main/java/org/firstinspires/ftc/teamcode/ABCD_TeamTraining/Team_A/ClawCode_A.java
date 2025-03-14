package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_A;

import org.firstinspires.ftc.robotcore.external.State;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;
import android.graphics.Color;

public class ClawCode_A {

    private String currentState;
    NormalizedColorSensor colorSensor;
    Servo clawServo;
    final float[] hsvValues = new float[3];

    public ClawCode_A(HardwareMap hardwareMap, Telemetry telemetry) {
        currentState = "IDLE";
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        clawServo = hardwareMap.get(Servo.class, "IS");
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }
    }

    public void update() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        switch(currentState){
            case "IDLE":
                clawServo.setPosition(1);
                //if color blue and color distance less than 2cm move to grabbed
                //set pos to open
                if (hsvValues[0] >= 180) {
                    currentState = "GRABBED";
                }

                break;
            case "GRABBED":
                clawServo.setPosition(0);

                if (hsvValues[0] <= 180) {
                    currentState = "IDLE";
                }

                //if distance < 20cm to wall move to IDLE
                //set pos to closed

                break;
            default:
                //error message to telemetry

                break;

        }
    }
}
