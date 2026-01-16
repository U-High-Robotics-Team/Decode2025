package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.RobotState;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(name="Mk2DecodeAutoV2")
public class Mk2DecodeAutoV2 extends OpMode {

    RobotTargetV2[] targets = {
            new RobotTargetV2(0, 0, 0, 100, RobotStates.HOME),

            // Move Back, Shoot, Intake First Row:
            // new RobotTargetV2(6.365, -764.3257, -2.3661, 100, RobotStates.HOME),
            // new RobotTargetV2(6.365, -764.3257, -2.3661, 6, RobotStates.SHOOT),
            // new RobotTargetV2(111.275, -870.6038, -1.587, 2, RobotStates.INTAKE);
    };

    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    // Max Speeds
    final double WHEEL_SPEED_MAX = 1;
    final double INTAKE_INTAKE_SPEED_MAX = -0.8;
    final double INTAKE_SHOOT_SPEED_MAX = -0.5;
    final double CONTROL_INTAKE_SPEED_MAX = 0.5;
    final double CONTROL_SHOOT_SPEED_MAX = -0.3;
    final double SHOOTER_VELOCITY_MAX = -1650;

    //Shooter Threshold
    final double SHOOTER_VELO_THRESHOLD = -1650;

    // Inital Conditions
    double wheelSpeed = WHEEL_SPEED_MAX;
    double intakeSpeed = 0.0;
    double controlSpeed = 0.0;
    double shooterVelocity = 0.0;
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;

    GoBildaPinpointDriver odo;
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor intake;
    private DcMotor control;
    private DcMotorEx shooter;

    double addX = 0;
    double addY = 0;
    double addTheta = 0;
    double deltaX;
    double deltaY;
    double deltaHeading;

    double kP = 0.0022; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot

    private double integralSum = 0;
    double lastError = 0;
    double targetDuration = 0;

    private int currentTargetIndex = 0; // Keeps track of the current target

    public enum RobotStates {
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
        intake = hardwareMap.get(DcMotor.class, "intake");
        control = hardwareMap.get(DcMotor.class, "control");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // PIDF for velocity control
        PIDFCoefficients shooterPIDF = new PIDFCoefficients(
                200,   // P
                5,    // I
                1,    // D
                0    // F
        );
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, shooterPIDF);

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        odo.setOffsets(-72, -156, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // Pose2D startingPosition= new Pose2D(DistanceUnit.MM,-944.006, -1661.3379, AngleUnit.RADIANS, -2.365);
        // odo.setPosition(startingPosition);


        Pose2D startingPosition= new Pose2D(DistanceUnit.MM,0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }

    public void moveRobotTo(double targetX, double targetY, double targetHeading) {
        odo.update();

        // Getting current positions
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        wheelSpeed = WHEEL_SPEED_MAX;

        // Finding errors using current and targets
        this.deltaX = targetX - currentX;
        this.deltaY = targetY - currentY;
        this.deltaHeading = targetHeading - currentHeading;

        // Accounting for minor errors
        if (Math.abs(deltaY) < 0.5) {
            deltaY = 0;
        }
        if (Math.abs(deltaX) < 0.5) {
            deltaX = 0;
        }
        if (Math.abs(deltaHeading) < 0.001) {
            deltaHeading = 0;
        }

        // Inversing y-axis
        // deltaY = -deltaY;

        // Using proportional controller for power
        // double xPower = Math.max(Math.min(findPIDPower(deltaX), 1), -1);
        // double yPower = Math.max(Math.min(findPIDPower(deltaY), 1), -1);


        double xPower = deltaX * kP;
        double yPower = deltaY * kP;
        double turnPower = -deltaHeading;

        //double turnPower = -Math.max(Math.min(findPower(deltaHeading), 1), -1);


        // double xPower = deltaX * kP;
        // double yPower = deltaY * kP;
        // double turnPower = -Math.toDegrees(deltaHeading) * 0.09;



        // Negative currentHeading due to rotating global power counterclockwise
        double cosAngle = Math.cos(-currentHeading);
        double sinAngle = Math.sin(-currentHeading);

        // Using inverse rotational matrix
        double localX = xPower * cosAngle + yPower * sinAngle;
        double localY = -xPower * sinAngle + yPower * cosAngle;

        // Calculating individual wheel speeds
        double frontLeft = (localX + localY + turnPower) * wheelSpeed;
        double frontRight = (localX - localY - turnPower) * wheelSpeed;
        double backLeft = (localX - localY + turnPower) * wheelSpeed;
        double backRight = (localX + localY - turnPower) * wheelSpeed;

        // FLeft.setPower(frontLeft);
        // FRight.setPower(frontRight);
        // BLeft.setPower(backLeft);
        // BRight.setPower(backRight);

        // telemetry.addData("Turn Power: ", turnPower);
        // telemetry.addData("CurrentX: ", currentX);
        // telemetry.addData("CurrentY: ", currentY);
        // telemetry.addData("Current Heading: ", currentHeading);
        // telemetry.addData("DeltaX", deltaX);
        // telemetry.addData("DeltaY", deltaY);
        // telemetry.addData("DeltaHeading", deltaHeading);
        // telemetry.update();
    }

    public void stopMotors() {
        FLeft.setPower(0);
        FRight.setPower(0);
        BLeft.setPower(0);
        BRight.setPower(0);
    }


    public void stateMachine() {
        switch (currentState) {
            case HOME:
                intakeSpeed = 0.0;
                controlSpeed = 0.0;
                shooterVelocity = 0.0;

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
                shooterVelocity = 0.0;

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
                shooterVelocity = SHOOTER_VELOCITY_MAX;

                if(shooter.getVelocity() < SHOOTER_VELO_THRESHOLD){
                    controlSpeed = CONTROL_SHOOT_SPEED_MAX;
                }else if(shooter.getVelocity() > SHOOTER_VELO_THRESHOLD+20){
                    controlSpeed = CONTROL_INTAKE_SPEED_MAX;
                }

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
        shooter.setVelocity(shooterVelocity);
    }


    public double findPIDPower(double delta){
        double derivative = 0;
        double out = 0;
        double error = 0;

        error = delta;

        // rate of change of the error
        derivative = (delta - lastError) / timer.seconds();

        // sum of all error over time
        out = (kP * delta) + (kD * derivative);

        this.lastError = delta;

        return out;
    }

    @Override
    public void loop() {
        if (currentTargetIndex < targets.length) {
            RobotTargetV2 target = targets[currentTargetIndex];
            this.targetDuration = target.time;
            requestedState = target.state;

            if (timer.seconds() < targetDuration) {
                moveRobotTo(target.x, target.y, target.heading);
                stateMachine();
                moveControl();
                moveIntake();
                moveShooter();
            } else {
                currentTargetIndex++;
                timer.reset();
            }
        } else {
            stopMotors(); // Stop the robot once all targets are processed
        }

        // updating odo positions
        odo.update();

        // getting current positions
        Pose2D currentPosition = odo.getPosition();
        double currentX = currentPosition.getX(DistanceUnit.MM);
        double currentY = currentPosition.getY(DistanceUnit.MM);
        double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Current Heading", currentHeading);
        telemetry.addData("DeltaX", deltaX);
        telemetry.addData("DeltaY", deltaY);
        telemetry.addData("DeltaHeading", deltaHeading);
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.update();
    }
}
