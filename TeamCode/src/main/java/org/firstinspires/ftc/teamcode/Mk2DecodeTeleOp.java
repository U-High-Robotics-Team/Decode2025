package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Installing respective libraries for operations.
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.Arrays;

@TeleOp(name="Mk2DecodeTeleOp")
public class Mk2DecodeTeleOp extends OpMode {
    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    // Max Speeds
    final double WHEEL_SPEED_MAX = 1;
    final double INTAKE_INTAKE_SPEED_MAX = -0.5;
    final double INTAKE_SHOOT_SPEED_MAX = -1;
    final double CONTROL_INTAKE_SPEED_MAX = 0.5;
    final double CONTROL_SHOOT_SPEED_MAX = -1;
    final double SHOOTER_SPEED_MAX = -1;

    //Shooter Threshold
    final double SHOOTER_VELO_THRESHOLD = 67;

    //Husky Conditions
    // Initializing a private variable that can only be referenced.
    private static final double APRILTAG_REAL_WIDTH = 6.0; // Standard width of APRIL TAG
    private static final double FOCAL_LENGTH_PIXEL_CONSTANT = 600.0; // constant needs to be determined (pixel length)?

    // Inital Conditions
    double wheelSpeed = WHEEL_SPEED_MAX;
    double intakeSpeed = 0.0;
    double controlSpeed = 0.0;
    double shooterSpeed = 0.0;
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;

    //    GoBildaPinpointDriver odo;
//private HuskyLens huskyLens;
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor intake;
    private DcMotor control;
    private DcMotorEx shooter;

    public enum RobotStates {
        HOME,
        INTAKE,
        SHOOT
    }

    @Override
    public void init() {
//        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
//        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
//        try {
//            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
//        } catch (Exception e) {
//            telemetry.addData("Error:", "HuskyLens not connected or algorithm failed to select.");
//        }
        telemetry.addData("Status", "HuskyLens Initialized. Waiting for Start.");
        telemetry.update();

        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        intake = hardwareMap.get(DcMotor.class, "intake");
        control = hardwareMap.get(DcMotor.class, "control");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // odometry configuration
//        odo.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        odo.resetPosAndIMU();
//
//        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
//        odo.setPosition(startingPosition);

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }

//
//    public void moveRobot() {
//        double forward = gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
//        double strafe = -gamepad1.left_stick_x * wheelSpeed;
//        double rotate = gamepad1.right_stick_x * wheelSpeed;
//
//        if (gamepad1.right_trigger > 0.9) {
//            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
//        }
//
//        Pose2D pos = odo.getPosition();
//        double heading = pos.getHeading(AngleUnit.RADIANS);
//
//        double cosAngle = Math.cos((Math.PI / 2) - heading);
//        double sinAngle = Math.sin((Math.PI / 2) - heading);
//
//        double globalForward = forward * cosAngle + strafe * sinAngle;
//        double globalStrafe = -forward * sinAngle + strafe * cosAngle;
//
//        double[] newWheelSpeeds = new double[4];
//
//        newWheelSpeeds[0] = globalForward - globalStrafe + rotate;
//        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
//        newWheelSpeeds[2] = globalForward + globalStrafe + rotate;
//        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;
//
//        FLeft.setPower(newWheelSpeeds[0]);
//        FRight.setPower(newWheelSpeeds[1]);
//        BLeft.setPower(newWheelSpeeds[2]);
//        BRight.setPower(newWheelSpeeds[3]);
//        //telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
//        //telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
//        //telemetry.addData("Robot Heading: ", heading);
//        //telemetry.addData("Forward Speed : ", globalForward);
//        //telemetry.addData("Strafe Speed : ", globalStrafe);
//
//        //telemetry.addData("Forward Speed : ", globalForward);
//        //telemetry.addData("Strafe Speed : ", globalStrafe);
//    }

    public void gamepadInputs() {
        // Switch Between Robot Modes: Shooting and Intaking
        if (gamepad2.right_trigger > 0.5) {
            requestedState = RobotStates.SHOOT;
        } else if (gamepad2.left_trigger > 0.5) {
            requestedState = RobotStates.INTAKE;
        } else if (gamepad2.start) {
            requestedState = RobotStates.HOME;
        }
    }

    public void stateMachine() {
        switch (currentState) {
            case HOME:
                intakeSpeed = 0.0;
                controlSpeed = 0.0;
                shooterSpeed = 0.0;

                if (requestedState == RobotStates.INTAKE) {
                    currentState = RobotStates.INTAKE;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT) {
                    currentState = RobotStates.SHOOT;
                    timer.reset();
                }

                break;

            case INTAKE:
                intakeSpeed = INTAKE_INTAKE_SPEED_MAX;
                controlSpeed = CONTROL_INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT) {
                    currentState = RobotStates.SHOOT;
                    timer.reset();
                }

                break;


            case SHOOT:
                intakeSpeed = INTAKE_SHOOT_SPEED_MAX;
                controlSpeed = CONTROL_SHOOT_SPEED_MAX;
                shooterSpeed = SHOOTER_SPEED_MAX;

//                if(shooter.getVelocity() > SHOOTER_VELO_THRESHOLD){
//                    controlSpeed = CONTROL_SHOOT_SPEED_MAX;
//                }else if(shooter.getVelocity() < SHOOTER_VELO_THRESHOLD){
//                    controlSpeed = CONTROL_INTAKE_SPEED_MAX;
//                }

                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE) {
                    currentState = RobotStates.INTAKE;
                    timer.reset();
                }

                break;
        }
    }

    public void moveIntake() {
        intake.setPower(intakeSpeed);
    }

    public void moveControl() {
        control.setPower(controlSpeed);
    }

    public void moveShooter() {
        shooter.setPower(shooterSpeed);
    }

    @Override
    public void loop() {
//        odo.update();

        gamepadInputs();
        stateMachine();
        moveIntake();
        moveControl();
//        moveRobot();
        moveShooter();

        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.update();
    }
}