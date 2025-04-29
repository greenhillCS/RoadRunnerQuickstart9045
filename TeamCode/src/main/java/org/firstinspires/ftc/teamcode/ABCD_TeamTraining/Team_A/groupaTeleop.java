package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@TeleOp(name = "#jaundicesasdfasdfaddf")
public class groupaTeleop extends LinearOpMode {

    private double power = 1;
    DcMotor motor;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor1;
    DistanceSensor distanceSensor2;
    final float[] hsvValues = new float[3];


    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "ds1");
        distanceSensor2 = hardwareMap.get(DistanceSensor.class, "ds2");
        // colorSensor = hardwareMap.get(NormalizedColorSensor .class, "sensor_color");
        DcMotor motor = hardwareMap.get(DcMotor.class, "motor");

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("dih", distanceSensor2.getDistance(DistanceUnit.CM));
            telemetry.update();
            motor.setPower(gamepad1.right_stick_y*power);

        }
    }
}
