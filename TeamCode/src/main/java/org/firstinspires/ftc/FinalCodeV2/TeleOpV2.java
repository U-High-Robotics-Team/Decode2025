package org.firstinspires.ftc.teamcode.OfficalCode;

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
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Installing respective libraries for operations.
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

import java.util.Arrays;


// 1 = GPP
// 2 = PGP
// 3 = PPG

// 0 = empty
// 1 = purple
// 2 = green

@TeleOp(name="DecodeTeleOp")
public class DecodeTeleOp extends OpMode {
    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();

    final double WHEEL_SPEED_MAX = 1;

    NormalizedRGBA colors;

    final double INTAKE_SPEED_MAX = -1;
    final double SHOOTER_SPEED_MAX = -1;

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

    //Husky params
    private HuskyLens huskyLens; // Initializing a private variable that can only be referenced.
    private static final double APRILTAG_REAL_WIDTH = 6.0; // Standard width of APRIL TAG
    private static final double FOCAL_LENGTH_PIXEL_CONSTANT = 600.0; // constant needs to be determined (pixel length)?

    // Inital Conditions
    boolean readyToShoot = false;
    double wheelSpeed = WHEEL_SPEED_MAX;
    double revolverTarget = SHOOT_1;
    double intakeSpeed = 0.0;
    double shooterSpeed = 0.0;
    double liftTarget = DOWN_LIFT;
    RobotStates currentState = RobotStates.HOME;
    RobotStates requestedState = RobotStates.HOME;
    int[] intakeStorage = new int[3];

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
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        try {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        } catch (Exception e) {
            telemetry.addData("Error:", "HuskyLens not connected or algorithm failed to select.");
        }
        telemetry.addData("Status", "HuskyLens Initialized. Waiting for Start.");
        telemetry.update();

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


        this.colors = colorSensor.getNormalizedColors();

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry configuration
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, -923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        //telemetry.addData("Status", "Initialized");
        //telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        //telemetry.addData("Device Scalar", odo.getYawScalar());
        //telemetry.update();

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }


    public void moveRobot() {
        double forward = gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        double strafe = -gamepad1.left_stick_x * wheelSpeed;
        double rotate = gamepad1.right_stick_x * wheelSpeed;

        // if (gamepad1.right_trigger > 0.9) {
        //     odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        // }

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
        //telemetry.addData("Robot XPos: ", pos.getX(DistanceUnit.MM));
        //telemetry.addData("Robot YPos: ", pos.getY(DistanceUnit.MM));
        //telemetry.addData("Robot Heading: ", heading);
        //telemetry.addData("Forward Speed : ", globalForward);
        //telemetry.addData("Strafe Speed : ", globalStrafe);

        //telemetry.addData("Forward Speed : ", globalForward);
        //telemetry.addData("Strafe Speed : ", globalStrafe);
    }

    public void gamepadInputs() {
        // Switch Between Robot Modes: Shooting and Intaking
        if (gamepad1.right_trigger > 0.5) {
            requestedState = nextStateForIntake();
        } else if (gamepad1.left_trigger > 0.5) {
            requestedState = nextStateForShoot();
        } else if (gamepad1.start) {
            requestedState = RobotStates.HOME;
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
        //telemetry.addData("Shooter RPM Target", targetRPM);
        //telemetry.addData("Shooter Velocity (t/s)", shooter.getVelocity());
        //telemetry.addData("Shooter RPM Actual", shooter.getVelocity() * 60 / ticksPerRev);
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

    public int availableShoot(){
        for (int i = 0; i < intakeStorage.length; i++) {
            if(this.intakeStorage[i] != 0){
                return i;
            }
        }

        return -1;
    }

    public RobotStates nextStateForShoot(){
        if(availableShoot() == 0){
            return RobotStates.SHOOT1;
        }else if(availableShoot() == 1){
            return RobotStates.SHOOT2;
        }else if(availableShoot() == 2){
            return RobotStates.SHOOT3;
        }else{
            return RobotStates.HOME;
        }
    }



    public void huskyLens() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        String detectedPattern = "None detected";

        if (blocks != null && blocks.length > 0) {
            telemetry.addData("AprilTags Detected", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                HuskyLens.Block block = blocks[i];
                telemetry.addData("Tag " + i + " ID", block.id);
                // ... (rest of the telemetry data) ...

                if (block.width > 0) {
                    double distance = (APRILTAG_REAL_WIDTH * FOCAL_LENGTH_PIXEL_CONSTANT) / block.width;
                    telemetry.addData("Tag " + i + " Distance (approx)", "%.2f cm", distance);
                }

                switch (block.id) {
                    case 1:
                        detectedPattern = "GPP";
                        break;
                    case 2:
                        detectedPattern = "PGP";
                        break;
                    case 3:
                        detectedPattern = "PPG";
                        break;
                    default:
                        detectedPattern = "Unknown ID";
                        break;
                }
            }
        } else {
            telemetry.addData("AprilTags Detected", "None");
        }

        telemetry.addData("Obelisk Pattern", detectedPattern);
        // telemetry.update() is handled by the main loop() method below, remove this specific one
    }




    @Override
    public void loop() {
        odo.update();
        this.colors = colorSensor.getNormalizedColors();

        gamepadInputs();
        stateMachine();

        moveRevolver();
        moveIntake();
        moveShooter();
        moveLift();
        moveRobot();
        huskyLens();

        //telemetry.addData("Current State", currentState);
        //telemetry.addData("Requested State", requestedState);
        //telemetry.addData("Ball Storage", Arrays.toString(intakeStorage));
        //telemetry.addData("Color Seen", colorSeen());
        //telemetry.addData("Timer (s)", timer.seconds());
        //telemetry.addData("Red", colors.red);
        //telemetry.addData("Blue", colors.blue);
        //telemetry.addData("Green", colors.green);

        telemetry.update();
    }

}
