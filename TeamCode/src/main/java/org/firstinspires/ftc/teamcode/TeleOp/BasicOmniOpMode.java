package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.FourBar;
import org.firstinspires.ftc.teamcode.Config.Intake;
import org.firstinspires.ftc.teamcode.Config.LinearActuator;
import org.firstinspires.ftc.teamcode.Config.PixelRelease;
import org.firstinspires.ftc.teamcode.Config.SlideController;
import org.firstinspires.ftc.teamcode.Config.droneLauncher;

@TeleOp(name="Correct OpMode", group="Linear OpMode")
public class BasicOmniOpMode extends LinearOpMode {

    private Integer Stopped = 0;
    private final ElapsedTime runtime = new ElapsedTime();

    // Constants for acceleration and maximum speed
    private static final double ACCELERATION = 0.5;
    private static final double MAX_SPEED = 0.75; // Adjust this value for your desired speed

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;
    private double accelerate(double currentPower, double targetPower, double acceleration){
        if (currentPower < targetPower) {
            return Math.min(currentPower + acceleration, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - acceleration, targetPower);
        }
        return targetPower;
    }

    @Override
    public void runOpMode() {

        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor slideMotor = hardwareMap.get(DcMotor.class, "SM");  // Replace "YourLinearSlideMotorName" with the actual name of your linear slide motor
        // Include the name of your linear slide motor here
//        DcMotor intakeMotor = hardwareMap.get(DcMotor.class, "IM");
        // Change to SC
        Servo droneServo = hardwareMap.get(Servo.class, "DS");  // Change to "SC"
//        DcMotor actuatorMotor = hardwareMap.get(DcMotor.class, "AM");
//        DcMotor fourBarMotor = hardwareMap.get(DcMotor.class, "FM");
        Servo pixelServo = hardwareMap.get(Servo.class, "PS");
        Servo wallServo = hardwareMap.get(Servo.class, "WS");
        DcMotor droneMotor = hardwareMap.get(DcMotor.class, "DM");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();


        SlideController slideController = new SlideController(slideMotor, gamepad2);
//        Intake intakeSystem = new Intake();
//        LinearActuator actuatorSystem = new LinearActuator();
//        FourBar fourBarSystem = new FourBar();
        PixelRelease pixelReleaser = new PixelRelease();
        droneLauncher droneLaunch = new droneLauncher();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Calculate the target powers for each wheel
            double targetLeftFrontPower = axial + lateral + yaw;
            double targetRightFrontPower = axial - lateral - yaw;
            double targetLeftBackPower = axial - lateral + yaw;
            double targetRightBackPower = axial + lateral - yaw;

            // Apply acceleration to the wheel powers
            leftFrontPower = accelerate(leftFrontPower, targetLeftFrontPower, ACCELERATION);
            rightFrontPower = accelerate(rightFrontPower, targetRightFrontPower, ACCELERATION);
            leftBackPower = accelerate(leftBackPower, targetLeftBackPower, ACCELERATION);
            rightBackPower = accelerate(rightBackPower, targetRightBackPower, ACCELERATION);

            // Normalize the values so no wheel power exceeds 100%
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }
//            if (Stopped <= 5 && axial + lateral + yaw == 0){
//                leftFrontPower = -1;
//                rightFrontPower = -1;
//                leftBackPower = -1;
//                rightBackPower = -1;
//                Stopped ++;
//            } else if(Stopped >= 5 && axial + lateral + yaw == 0) {
//                leftFrontPower = 0;
//                rightFrontPower = 0;
//                leftBackPower = 0;
//                rightBackPower = 0;
//            }
//            if (Stopped >= 5 && axial + lateral + yaw != 0){
//                Stopped = 0;
//            }

            // Set motor powers with the reduced and accelerated values
            leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
            rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
            leftBackDrive.setPower(leftBackPower * MAX_SPEED);
            rightBackDrive.setPower(rightBackPower * MAX_SPEED);

            slideController.update();
            // slideController.updateMacros();// Control the slide motor

            if (gamepad2.x) {
                droneLaunch.shoot(droneServo);
            } else if (gamepad2.y) {
                droneLaunch.load(droneServo);
            }

            // if (gamepad2.dpad_up){
            //     actuatorSystem.forward(actuatorMotor);


            // }
            // else if (gamepad2.dpad_down){
            //     //actuatorSystem.reverse(actuatorMotor);
            //     fourBarSystem.fullDown(fourBarMotor);
            // }
            // else if (gamepad2.dpad_left){
            //     //actuatorSystem.reverse(actuatorMotor);
            //     fourBarSystem.startPos(fourBarMotor);
            // }
            // else if (gamepad2.dpad_right){
            //     //actuatorSystem.reverse(actuatorMotor);
            //     fourBarSystem.wallPos(fourBarMotor);
            // }
            if (gamepad2.right_bumper) {
                droneLaunch.droneOut(droneMotor);
            } else if (gamepad2.left_bumper) {
                droneLaunch.droneIn(droneMotor);
            } else {
                droneLaunch.droneStop(droneMotor);
            }
            if (gamepad2.a) {
                pixelReleaser.close(pixelServo);
                pixelReleaser.close(wallServo);
            } else if (gamepad2.b) {
                pixelReleaser.open(pixelServo);
                pixelReleaser.open(wallServo);
            }


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }

        // Helper function to apply acceleration

    }
}