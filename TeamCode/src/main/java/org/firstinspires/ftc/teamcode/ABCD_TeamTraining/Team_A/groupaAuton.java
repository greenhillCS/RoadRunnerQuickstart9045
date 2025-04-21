package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "#robot b code")
public class groupaAuton extends LinearOpMode {

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
        motor.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {

            telemetry.addData("d2", distanceSensor2.getDistance(DistanceUnit.CM));
            telemetry.addData("d1", distanceSensor1.getDistance(DistanceUnit.CM));
            telemetry.update();
            if (distanceSensor2.getDistance(DistanceUnit.CM) < 25){
                motor.setPower(0.5);
                telemetry.addData("Status", "should reverse");
            } else if ((distanceSensor1.getDistance(DistanceUnit.CM) < 25)) {
                motor.setPower(-0.5);
            } else {
                telemetry.addData("Status", "normal");
            }

        }
    }
}
