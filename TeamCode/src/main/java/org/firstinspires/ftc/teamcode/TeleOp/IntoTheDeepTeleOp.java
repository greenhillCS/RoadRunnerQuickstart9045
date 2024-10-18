package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auton.Position.PositionStorage;
import org.firstinspires.ftc.teamcode.Config.IntoTheDeepSlides;
import org.firstinspires.ftc.teamcode.Config.PixelRelease;
import org.firstinspires.ftc.teamcode.drive.Constants.Config.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group="intothedeep", name="IntoTheDeepTeleOp")
public class IntoTheDeepTeleOp extends LinearOpMode {
    enum Mode {
        DRIVER_CONTROL,
        AUTOMATIC_CONTROL
    }

    Mode currentMode = Mode.DRIVER_CONTROL;

    private static final double ACCELERATION = 0.2;
    private static final double MAX_SPEED = 0.75; // Adjust this value for your desired speed

    private double leftFrontPower = 0;
    private double rightFrontPower = 0;
    private double leftBackPower = 0;
    private double rightBackPower = 0;

    public double x = 0,y = 0, h = 0;

    private ElapsedTime time = new ElapsedTime();

    PixelRelease clawController;
    Servo clawServo;

    IntoTheDeepSlides scorer;
    IntoTheDeepSlides hangar;

    private double accelerate(double currentPower, double targetPower, double acceleration){
        if (currentPower < targetPower) {
            return Math.min(currentPower + acceleration, targetPower);
        } else if (currentPower > targetPower) {
            return Math.max(currentPower - acceleration, targetPower);
        }
        return targetPower;
    }
    private final ElapsedTime runtime = new ElapsedTime();

    private void moveClawWait(int pos){
        scorer.emergencyStop();
        clawServo.setPosition(pos);
        time.reset();
        while(opModeIsActive() && time.seconds() < 0.5){
            continue;
        }
        if (pos==1) {
            scorer.hookPosUp();
        } else {
            scorer.startPos();
        }
    }


    @Override
    public void runOpMode (){

        //Mecanum Drive Class init
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(PositionStorage.pose);


        //Drive Motor init
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //DC Motor init
        DcMotor scoringMotor = hardwareMap.get(DcMotor.class, "SM");
        DcMotor hangingMotor = hardwareMap.get(DcMotor.class, "HM");

        //Servo Motor init
        clawServo = hardwareMap.get(Servo.class, "CS");
        clawController = new PixelRelease();

        scorer = new IntoTheDeepSlides(scoringMotor);
        hangar = new IntoTheDeepSlides(hangingMotor);//WHAT THE SIGMA

        waitForStart();

        //Drive Motor direction init
        while (opModeIsActive()) {


            //DRIVE CONTROLS vvvvv
            switch (currentMode) {
                case DRIVER_CONTROL:
                    x = drive.getPoseEstimate().getX();
                    y = drive.getPoseEstimate().getY();
                    h = drive.getPoseEstimate().getHeading();

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

                    //HANGING SLIDE CONTROLS vvvvv
                    if (gamepad1.dpad_left) {
                        hangar.startPos();
                    } else if (gamepad1.right_trigger > 0) {
                        hangar.up(gamepad1.right_trigger);
                    } else if (gamepad1.left_trigger > 0) {
                        hangar.down(gamepad1.left_trigger);
                    } else if (gamepad1.dpad_down) {
                        hangar.hardPull();
                    } else {
                        hangar.stop();
                    }
                    //HANGING SLIDE CONTROLS ^^^^^

                    //SCORING SLIDE CONTROLS vvvvv
                    if (gamepad2.a && !gamepad2.start) {
                        moveClawWait(0);
                    } else if (gamepad2.y) {
                        moveClawWait(1);
                    } else if (gamepad2.right_trigger > 0) {
                        scorer.emergencyStop();
                        scorer.up(gamepad2.right_trigger);
                    } else if (gamepad2.left_trigger > 0) {
                        scorer.emergencyStop();
                        scorer.down(gamepad2.left_trigger);
                    } else if (gamepad2.dpad_down) {
                        scorer.emergencyStop();
                        scorer.hardPull();
                    } else {
                        scorer.stop();
                    }
                    //SCORING SLIDE CONTROLS ^^^^^

                    //CLAW CONTROLS vvvvv
                    if (gamepad2.right_bumper) {
                        scorer.emergencyStop();
                        clawServo.setPosition(1);
                    } else if (gamepad2.left_bumper) {
                        scorer.emergencyStop();
                        clawServo.setPosition(0);
                    }
                    //CLAW CONTROLS ^^^^^

                    //WAYPOINT CONTROLS vvvvv
                    if (gamepad1.a && !gamepad1.start) {
                        telemetry.addData("Moving To:", "Ascent Zone");
                        if (y <= -24){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(0)))
                                    .build());
                        }
                        if (y <= 24){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, 36, Math.toRadians(0)))
                                    .build());
                        }
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .splineToLinearHeading(new Pose2d(-22.5, -12, h), Math.toRadians(0))
                                .build());
                        while(gamepad1.a){
                            continue;
                        }
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.b && !gamepad1.start) {
                        telemetry.addData("Moving To:", "Net Zone");
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(-51.5, -53, Math.toRadians(225)))
                                .build());
                        while(gamepad1.b){
                            continue;
                        }
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.x) {
                        telemetry.addData("Moving To:", "Submersible Zone");
                        if (y >= -36 && x < 0){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-36, -36, Math.toRadians(90)))
                                    .build());
                        } else if (y >= -36 && x > 0){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(36, -36, Math.toRadians(90)))
                                    .build());
                        }
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(0, -32, Math.toRadians(90)))
                                .build());
                        while(gamepad1.x){
                            continue;
                        }
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.y) {
                        telemetry.addData("Moving To:", "Observation Zone");
                        if (y >= -24 && x < 0){
                            drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToSplineHeading(new Pose2d(-48, -36, Math.toRadians(225)))
                                    .build());
                        }
                        drive.followTrajectoryAsync(drive.trajectoryBuilder(drive.getPoseEstimate())
                                .lineToSplineHeading(new Pose2d(52, -50, Math.toRadians(270)))
                                .build());
                        while(gamepad1.y){
                            continue;
                        }
                        currentMode = Mode.AUTOMATIC_CONTROL;
                    } else if (gamepad1.dpad_up){
                        drive.setPoseEstimate(new Pose2d(0, -72+(DriveConstants.BOT_LENGTH/2), Math.toRadians(90)));
                    } else {
                        leftFrontDrive.setPower(leftFrontPower * MAX_SPEED);
                        rightFrontDrive.setPower(rightFrontPower * MAX_SPEED);
                        leftBackDrive.setPower(leftBackPower * MAX_SPEED);
                        rightBackDrive.setPower(rightBackPower * MAX_SPEED);
                    }
                    break;
                case AUTOMATIC_CONTROL:
                    if(gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y){
                        drive.breakFollowing();
                        while (gamepad1.a||gamepad1.b||gamepad1.x||gamepad1.y){
                            continue;
                        }
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    if (!drive.isBusy()) {
                        currentMode = Mode.DRIVER_CONTROL;
                    }
                    break;
            }
                //WAYPOINT CONTROLS ^^^^^}

            drive.update();

            Pose2d pos = drive.getPoseEstimate();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("X", pos.getX());
            telemetry.addData("Y", pos.getY());
            telemetry.addData("Right Trigger", gamepad2.right_trigger);
            telemetry.addData("Heading", Math.toDegrees(pos.getHeading()));
            telemetry.update();
        }


    }

}


