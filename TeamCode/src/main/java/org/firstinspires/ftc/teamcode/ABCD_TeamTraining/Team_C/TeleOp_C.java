package org.firstinspires.ftc.teamcode.ABCD_TeamTraining.Team_C;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.IntoTheDeepIntakeSystem;

@TeleOp(name="Team C TeleOp", group="ABCD TeleOps")
public class TeleOp_C extends LinearOpMode {

    private static final double ACCELERATION = 0.2;
    private static final double MAX_SPEED = 1.0; // Adjust this value for your desired speed

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    public double x = 0,y = 0, h = 0;

    IntoTheDeepIntakeSystem intake;
    ClawCode_C clawCode;

    Servo intakeServo;
    Servo rotationServo;
    Servo angleServo;
    void clawControl(double angle, double rotation){
        angleServo.setPosition(angle);
        rotationServo.setPosition(rotation);
    }
    private double accelerate(double currentPower, double targetPower, double acceleration){
        if (currentPower < targetPower) {
            return Math.min(currentPower + acceleration, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - acceleration, targetPower);
        }
        return targetPower;
    }
    private final ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode (){

        //Drive Motor init
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");//port 1
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");//port 3
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");//port 0
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");//port 2
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //DC Motor init
        DcMotor jointMotor = hardwareMap.get(DcMotorEx.class, "JM");//port 2
        DcMotor intakeMotor = hardwareMap.get(DcMotorEx.class, "IM");//port 3

        //Servo Motor init
        intakeServo = hardwareMap.get(Servo.class, "IS");//port 1
        rotationServo = hardwareMap.get(Servo.class, "RS");//port 3
        angleServo = hardwareMap.get(Servo.class, "AS");//port 2

        RevTouchSensor intakeSlidesTouchSensor = hardwareMap.get(RevTouchSensor.class, "TI");//port 3
        RevTouchSensor angleTouchSensor = hardwareMap.get(RevTouchSensor.class, "TA");//port 5

        intake = new IntoTheDeepIntakeSystem(intakeMotor, jointMotor, intakeSlidesTouchSensor, angleTouchSensor, telemetry);//WHAT THE SIGMA
        clawCode = new ClawCode_C(hardwareMap, telemetry);

        waitForStart();

        //Drive Motor direction init
        while (opModeIsActive()) {

            //Set's the motor's powers
            leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
            rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
            leftBackDrive.setPower(leftBackPower * MAX_SPEED);
            rightBackDrive.setPower(rightBackPower * MAX_SPEED);

            //DRIVE CONTROLS vvvvv
            double max;
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
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
            //DRIVE CONTROLS ^^^^^

            //INTAKE CONTROLS vvvvv
            if (gamepad2.left_bumper){
                //open claw
                intakeServo.setPosition(1);
            }

            if (gamepad2.dpad_up){
                //yaw up
                angleServo.setPosition(Math.min(angleServo.getPosition() + 0.01, 1));
            }if (gamepad2.dpad_down){
                //yaw down
                angleServo.setPosition(Math.max(angleServo.getPosition() - 0.01, 0));
            }

            if (gamepad2.dpad_left){
                //roll left
                rotationServo.setPosition(Math.min(rotationServo.getPosition() + 0.01, 1));
            }if (gamepad2.dpad_right){
                //roll right
                rotationServo.setPosition(Math.max(rotationServo.getPosition() - 0.01, 0));
            }
            if(gamepad2.b && !gamepad2.start){
                //moves intake system to grab from the wall
                intake.moveTo(0, -4000);
                angleServo.setPosition(0.7);
                rotationServo.setPosition(0.3);
            }else if(gamepad2.y){
                //moves intake system to score
                intake.moveTo(900, -850);
                angleServo.setPosition(0.7);
                rotationServo.setPosition(1);
            } else if(gamepad2.x){
                //moves intake system to pick up from the submersible
                intake.moveTo(400, -4540);
                angleServo.setPosition(0.43);
                rotationServo.setPosition(0.3);
            }else if(gamepad2.back){
                intake.startPos();
                clawControl(0, 0.3);
            }
            //updates intake slides, angler, and automatic claw closing controls
            clawCode.update();
            intake.update(gamepad2);
            //INTAKE CONTROLS ^^^^^


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Joint Encoder", jointMotor.getCurrentPosition());
            telemetry.addData("Joint Power", jointMotor.getPower());
            telemetry.addData("Intake Encoder", intakeMotor.getCurrentPosition());
            telemetry.addData("Intake Power", intakeMotor.getPower());
            telemetry.addData("Claw Pos", intakeServo.getPosition());
            telemetry.addData("Rotation Pos", rotationServo.getPosition());
            telemetry.addData("Angle Pos", angleServo.getPosition());
            telemetry.update();
        }

    }

}