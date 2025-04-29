package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_A;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "..Distance Spinner")
public class distanceSpinner extends LinearOpMode {
    private double power = 0;
    private double distance;
    private double scale = 0.01;
    DistanceSensor distanceSensor1;
    DcMotor motor;
    DcMotor motor2;
    Servo servo;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensor1 = hardwareMap.get(DistanceSensor.class, "ds");
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        servo = hardwareMap.get(Servo.class, "servo");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            distance = distanceSensor1.getDistance(DistanceUnit.CM);
            power = distance*scale;
            telemetry.addData("distance", distance);
            telemetry.addData("power", power);
            motor.setPower(power);
            motor2.setPower(power);
            if (gamepad1.a) {
                servo.setPosition(1);
            }
            if (gamepad1.b){
                servo.setPosition(0);
            }
        }
    }
}
