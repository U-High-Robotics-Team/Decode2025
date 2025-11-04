package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
// import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="GlobalTeleOp")
public class GlobalTeleOp extends OpMode {
    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;


    // Revolving Servo Positions
    final double INTAKE_1 = 0.0;
    final double INTAKE_2 = 0.45;
    final double INTAKE_3 = 0.9;
    final double SHOOT_1 = 0.2;
    final double SHOOT_2 = .65;
    final double SHOOT_3 = 1; // not correct

    // Inital Conditions
    boolean readyToShoot = false;
    double wheelSpeed = WHEEL_SPEED_MAX;
    double revolverTarget = SHOOT_1;
    double intakeSpeed = 0.0;
    double shooterSpeed = 0.0;
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;


    GoBildaPinpointDriver odo;
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor intake;
    private DcMotor shooter;
    private Servo revolver;

    enum RobotStates {
        HOME,
        INTAKE,
        SHOOT
    }


    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");
        revolver = hardwareMap.get(Servo.class, "revolver");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");


        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry configuration
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);


        telemetry.addData("Status", "Initialized");
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }


    public void moveRobot() {
        double forward = -gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        double strafe = gamepad1.left_stick_x * wheelSpeed;
        double rotate = gamepad1.right_stick_x * wheelSpeed;

        if (gamepad1.right_trigger > 0.9) {
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        Pose2D pos = odo.getPosition();
        double heading = pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2) - heading);
        double sinAngle = Math.sin((Math.PI / 2) - heading);

        double globalForward = forward * cosAngle + strafe * sinAngle;
        double globalStrafe = -forward * sinAngle + strafe * cosAngle;

        double[] newWheelSpeeds = new double[4];

        newWheelSpeeds[0] = globalForward + globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[3] = globalForward + globalStrafe - rotate;

        FLeft.setPower(newWheelSpeeds[0]);
        FRight.setPower(newWheelSpeeds[1]);
        BLeft.setPower(newWheelSpeeds[2]);
        BRight.setPower(newWheelSpeeds[3]);
        telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        telemetry.addData("Robot Heading: ", heading);
        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);

        telemetry.addData("Forward Speed : ", globalForward);
        telemetry.addData("Strafe Speed : ", globalStrafe);
    }

    public void gamepadInputs() {
        // Switch Between Robot Modes: Shooting and Intaking States
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
                shooterSpeed = 0.0;

                break;

            case INTAKE:
                shooterSpeed = 0.0;
                intakeSpeed = 1.0;

                if (gamepad2.a) {
                    revolverTarget = INTAKE_1;
                } else if (gamepad2.b) {
                    revolverTarget = INTAKE_2;
                } else if (gamepad2.y) {
                    revolverTarget = INTAKE_3;
                }

                break;

            case SHOOT:
                shooterSpeed = 1.0;
                intakeSpeed = 0.0;

                if (gamepad2.a) {
                    revolverTarget = SHOOT_1;
                } else if (gamepad2.b) {
                    revolverTarget = SHOOT_2;
                } else if (gamepad2.y) {
                    revolverTarget = SHOOT_3;
                }

                break;
        }
    }

    public void moveRevolver() {
        revolver.setPosition(revolverTarget);
    }

    public void moveIntake() {
        intake.setPower(intakeSpeed);
    }

    public void moveShooter() {
        shooter.setPower(shooterSpeed);
    }

    @Override
    public void loop() {
        gamepadInputs();
        moveRobot();
        moveRevolver();
        moveIntake();
        moveShooter();
        stateMachine();

        odo.update();

        telemetry.addData("Ready to Shoot", this.readyToShoot);
        telemetry.update();
    }
}