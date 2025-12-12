package org.firstinspires.ftc.teamcode.OfficalCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Arrays;

@TeleOp(name="FinalDecodeTeleOp")
public class FinalDecodeTeleOp extends OpMode {

    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();
    public final double TIME_BTW_REVOLVE_AND_LIFT = 1.5;
    public final double TIME_BTW_LIFT_AND_DOWN = 3;
    public final double TIME_BTW_DOWN_AND_REVOLVE = 3.5;


    // Logic Variables
    public boolean isManual = false;
    NormalizedRGBA colors;

    // Speed Max
    final double INTAKE_SPEED_MAX = -1;
    final double SHOOTER_SPEED_MAX = -1;
    final double WHEEL_SPEED_MAX = 1;
    int[] order;

    // Speed Min
    final double INTAKE_SPEED_MIN = 0;
    final double SHOOTER_SPEED_MIN = 0;

    // Revolving Servo Positions
    final double INTAKE_1 = 0.207;
    final double INTAKE_2 = 0.27;
    final double INTAKE_3 = 0.337;
    final double SHOOT_1 = 0.305;
    final double SHOOT_2 = 0.37;
    final double SHOOT_3 = 0.447;

    // Lift Servo Positions
    private final double UP_LIFT = 0.89;
    private final double DOWN_LIFT = 0.85;

    // Inital Conditions
    double wheelSpeed = WHEEL_SPEED_MAX;
    double revolverTarget = SHOOT_1;
    double intakeSpeed = 0.0;
    double shooterSpeed = 0.0;
    double liftTarget = DOWN_LIFT;
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;
    int[] intakeStorage = new int[3];
    int[] targetStorage = new int[3];

    int desiredTarget;

    GoBildaPinpointDriver odo;
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;
    private DcMotor intake;
    private DcMotorEx shooter;
    private Servo revolver;
    private Servo lift;
    private NormalizedColorSensor colorSensor;

    public enum RobotStates {
        HOME,
        INTAKE1,
        INTAKE2,
        INTAKE3,
        SHOOT1,
        SHOOT2,
        SHOOT3
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
        lift = hardwareMap.get(Servo.class, "lift");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(10, 3, 0, 12)); // tune as needed

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry configuration
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        // inital conditions
        targetStorage[0] = 2;
        targetStorage[1] = 1;
        targetStorage[2] = 1;

        // TODO:remove
        intakeStorage[0] = 1;
        intakeStorage[1] = 1;
        intakeStorage[2] = 2;

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);
        this.colors = colorSensor.getNormalizedColors();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }


    public void moveRobot() {
        double forward = gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        double strafe = -gamepad1.left_stick_x * wheelSpeed;
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

        newWheelSpeeds[0] = globalForward - globalStrafe + rotate;
        newWheelSpeeds[1] = globalForward - globalStrafe - rotate;
        newWheelSpeeds[2] = globalForward + globalStrafe + rotate;
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
        // Switch Between Robot Modes: Shooting and Intaking
        if (gamepad2.right_trigger > 0.5) {
            requestedState = nextStateForIntake();
        } else if (gamepad2.left_trigger > 0.5) {
            requestedState = nextStateForShoot();
        } else if (gamepad2.right_bumper) {
            requestedState = RobotStates.HOME;
        }

        if (gamepad1.a) {
            // Set target order green, purple, purple
            targetStorage[0] = 2;
            targetStorage[1] = 1;
            targetStorage[2] = 1;
        } else if (gamepad1.b) {
            // Set target order purple, green, purple
            targetStorage[0] = 1;
            targetStorage[1] = 2;
            targetStorage[2] = 1;
        } else if (gamepad1.y) {
            // Set target order purple, purple, green
            targetStorage[0] = 1;
            targetStorage[1] = 1;
            targetStorage[2] = 2;
        }
    }

    public void stateMachine() {
        switch (currentState) {
            case HOME:
                intakeSpeed = 0.0;
                shooterSpeed = 0.0;

                if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case INTAKE1:
                revolverTarget = INTAKE_1;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                if(timer.seconds()>2 && intakeStorage[0] == 0){
                    intakeStorage[0] = colorSeen();

                    if(intakeStorage[0] != 0){
                        requestedState = nextStateForIntake();
                    }
                }

                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case INTAKE2:
                revolverTarget = INTAKE_2;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                if(timer.seconds()>2 && intakeStorage[1] == 0){
                    intakeStorage[1] = colorSeen();

                    if(intakeStorage[1] != 0){
                        requestedState = nextStateForIntake();
                    }
                }



                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case INTAKE3:
                revolverTarget = INTAKE_3;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                if(timer.seconds()>2 && intakeStorage[2] == 0){
                    intakeStorage[2] = colorSeen();

                    if(intakeStorage[2] != 0){
                        requestedState = nextStateForIntake();
                    }
                }

                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case SHOOT1:
                revolverTarget = SHOOT_1;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[0] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }

                if(intakeStorage[0] == 0 && liftTarget == DOWN_LIFT && timer.seconds()>3.5){
                    requestedState = nextStateForShoot();
                }


                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case SHOOT2:
                revolverTarget = SHOOT_2;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[1] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }

                if(intakeStorage[1] == 0 && liftTarget == DOWN_LIFT && timer.seconds()>3.5){
                    requestedState = nextStateForShoot();
                }


                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT3) {
                    currentState = RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case SHOOT3:
                revolverTarget = SHOOT_3;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;
                intakeStorage[2] = 0;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[2] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }

                if(intakeStorage[2] == 0 && liftTarget == DOWN_LIFT && timer.seconds()>3.5){
                    requestedState = nextStateForShoot();
                }


                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE2) {
                    currentState = RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE3) {
                    currentState = RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT1) {
                    currentState = RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState == RobotStates.SHOOT2) {
                    currentState = RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState == RobotStates.INTAKE1) {
                    currentState = RobotStates.INTAKE1;
                    timer.reset();
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
        double targetRPM = 1500;

        double ticksPerRev = 537.7; // GoBilda motor encoder CPR
        double targetVelocity = targetRPM * ticksPerRev / 60.0;

        if(shooterSpeed == SHOOTER_SPEED_MAX) {
            shooter.setVelocity(targetVelocity);
        }else{
            shooter.setVelocity(0);
        }
        telemetry.addData("Shooter RPM Target", targetRPM);
        telemetry.addData("Shooter Velocity (t/s)", shooter.getVelocity());
        telemetry.addData("Shooter RPM Actual", shooter.getVelocity() * 60 / ticksPerRev);
    }


    public void manualGamepadInputs() {
        // Switch Between Robot Modes: Shooting and Intaking
        if (gamepad2.right_trigger > 0.5 && gamepad2.a) {
            requestedState =  RobotStates.INTAKE1;
        } else if (gamepad2.right_trigger > 0.5 && gamepad2.b) {
            requestedState =  RobotStates.INTAKE2;
        } else if (gamepad2.right_trigger > 0.5 && gamepad2.y) {
            requestedState =  RobotStates.INTAKE3;
        } else if (gamepad2.left_trigger > 0.5 && gamepad2.a) {
            requestedState =  RobotStates.SHOOT1;
        } else if (gamepad2.left_trigger > 0.5 && gamepad2.b) {
            requestedState =  RobotStates.SHOOT2;
        } else if (gamepad2.left_trigger > 0.5 && gamepad2.y) {
            requestedState =  RobotStates.SHOOT3;
        } else if (gamepad2.start) {
            requestedState =  RobotStates.HOME;
        }
    }

    public void manualStateMachine() {
        switch (currentState) {
            case HOME:
                intakeSpeed = 0.0;
                shooterSpeed = 0.0;

                if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case INTAKE1:
                revolverTarget = INTAKE_1;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                intakeStorage[0] = colorSeen();

                if (requestedState ==  RobotStates.HOME) {
                    currentState =  RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case INTAKE2:
                revolverTarget = INTAKE_2;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;


                intakeStorage[1] = colorSeen();



                if (requestedState ==  RobotStates.HOME) {
                    currentState =  RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case INTAKE3:
                revolverTarget = INTAKE_3;
                intakeSpeed = INTAKE_SPEED_MAX;
                shooterSpeed = 0.0;

                intakeStorage[2] = colorSeen();


                if (requestedState ==  RobotStates.HOME) {
                    currentState =  RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case SHOOT1:
                revolverTarget = SHOOT_1;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[0] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }

                if (requestedState ==  RobotStates.HOME) {
                    currentState =  RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;
            case SHOOT2:
                revolverTarget = SHOOT_2;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[1] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }

                if (requestedState ==  RobotStates.HOME) {
                    currentState =  RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT3) {
                    currentState =  RobotStates.SHOOT3;
                    timer.reset();
                }

                break;

            case SHOOT3:
                revolverTarget = SHOOT_3;
                intakeSpeed = 0.0;
                shooterSpeed = SHOOTER_SPEED_MAX;
                intakeStorage[2] = 0;

                if(timer.seconds() > 1.5){
                    liftTarget = UP_LIFT;
                    intakeStorage[2] = 0;
                }

                if(timer.seconds()>3){
                    liftTarget = DOWN_LIFT;
                }


                if (requestedState == RobotStates.HOME) {
                    currentState = RobotStates.HOME;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE2) {
                    currentState =  RobotStates.INTAKE2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE3) {
                    currentState =  RobotStates.INTAKE3;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT1) {
                    currentState =  RobotStates.SHOOT1;
                    timer.reset();
                } else if (requestedState ==  RobotStates.SHOOT2) {
                    currentState =  RobotStates.SHOOT2;
                    timer.reset();
                } else if (requestedState ==  RobotStates.INTAKE1) {
                    currentState =  RobotStates.INTAKE1;
                    timer.reset();
                }

                break;
        }
    }


    public void moveLift() {
        lift.setPosition(liftTarget);
    }

    public int colorSeen(){
        this.colors = colorSensor.getNormalizedColors();

        if(colors.red < 0.01 & colors.green < 0.01 & colors.blue < 0.01){
            return 0;
        } else if(colors.green<colors.blue){
            return 1;
        } else if(colors.green>colors.blue){
            return 2;
        }

        // if this hits then the code has an error
        return -1;
    }

    public int availableIntake(){
        for (int i = 0; i < intakeStorage.length; i++) {
            if(this.intakeStorage[i] == 0){
                return i;
            }
        }

        return -1;
    }

    public RobotStates nextStateForIntake(){
        if(availableIntake() == 0){
            return RobotStates.INTAKE1;
        }else if(availableIntake() == 1){
            return RobotStates.INTAKE2;
        }else if(availableIntake() == 2){
            return RobotStates.INTAKE3;
        }else{
            return RobotStates.HOME;
        }
    }

//    public int availableShoot(){
//        for (int i = 0; i < intakeStorage.length; i++) {
//            if(this.intakeStorage[i] != 0){
//                return i;
//            }
//        }
//
//        return -1;
//    }
//
//    public RobotStates nextStateForShoot(){
//        if(availableShoot() == 0){
//            return RobotStates.SHOOT1;
//        }else if(availableShoot() == 1){
//            return RobotStates.SHOOT2;
//        }else if(availableShoot() == 2){
//            return RobotStates.SHOOT3;
//        }else{
//            return RobotStates.HOME;
//        }
//    }

    public RobotStates nextStateForShoot() {
        this.order = getShootOrder();

        // Find the first filled intake index according to that order
        for (int idx : this.order) {
            if (intakeStorage[idx] != 0) {
                switch (idx) {
                    case 0: return RobotStates.SHOOT1;
                    case 1: return RobotStates.SHOOT2;
                    case 2: return RobotStates.SHOOT3;
                }
            }
        }

        return RobotStates.HOME;
    }


    public int[] getShootOrder() {
        // returns index order (0–2) that should be shot next based on target priorities
        this.order = new int[3];
        boolean[] used = new boolean[3];

        for (int i = 0; i < 3; i++) {
            this.desiredTarget = targetStorage[i];
            int bestMatchIndex = -1;

            // find first intake slot matching that color that hasn't been used
            for (int j = 0; j < 3; j++) {
                if (!used[j] && intakeStorage[j] == desiredTarget) {
                    bestMatchIndex = j;
                    break;
                }
            }

//            // if no exact match, just fill with the next available non-empty slot
//            if (bestMatchIndex == -1) {
//                for (int j = 0; j < 3; j++) {
//                    if (!used[j] && intakeStorage[j] != 0) {
//                        bestMatchIndex = j;
//                        break;
//                    }
//                }
//            }

            // if still none found, fill with the first unused index
            if (bestMatchIndex == -1) {
                for (int j = 0; j < 3; j++) {
                    if (!used[j]) {
                        bestMatchIndex = j;
                        break;
                    }
                }
            }

            order[i] = bestMatchIndex;
            used[bestMatchIndex] = true;
        }

        return order;
    }


    @Override
    public void loop() {
        odo.update();
        this.colors = colorSensor.getNormalizedColors();


        // handles switching between manual and auto
        if(gamepad2.back) {
            isManual = true;
        }else if(gamepad2.start){
            isManual = false;
        }

        if(isManual){
            manualGamepadInputs();
            manualStateMachine();
        }else{
            gamepadInputs();
            stateMachine();
        }

        moveRevolver();
        moveIntake();
        moveShooter();
        moveLift();
        moveRobot();

        telemetry.addData("Current State", currentState);
        telemetry.addData("Requested State", requestedState);
        telemetry.addData("Ball Storage", Arrays.toString(intakeStorage));
        telemetry.addData("Target Storage", Arrays.toString(targetStorage));
        telemetry.addData("Order Target", Arrays.toString(order));
        telemetry.addData("Color Seen", colorSeen());
        telemetry.addData("Timer (s)", timer.seconds());
        telemetry.addData("Red", colors.red);
        telemetry.addData("Blue", colors.blue);
        telemetry.addData("Green", colors.green);
        telemetry.addData("Desired Color", desiredTarget);
        telemetry.update();
    }

}