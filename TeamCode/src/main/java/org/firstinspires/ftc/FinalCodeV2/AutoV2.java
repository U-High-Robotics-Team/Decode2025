package org.firstinspires.ftc.teamcode.OfficalCode;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.OfficalCode.DecodeTeleOp.RobotStates;
import org.firstinspires.ftc.teamcode.OfficalCode.RobotTarget;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

@Autonomous(name="CloseAutonomous")
public class CloseAutonomous extends OpMode {

    RobotTarget[] targets = {
            new RobotTarget(1750, 0, 0, 4, RobotStates.HOME),
            new RobotTarget(1750, 0, 0, 4, RobotStates.SHOOT1),
            new RobotTarget(1750, 0, 0, 4, RobotStates.SHOOT2),
            new RobotTarget(1750, 0, 0, 4, RobotStates.SHOOT3),
            new RobotTarget(1500, 0, 0, 4,RobotStates.INTAKE1)
    };

    // Timer for Servos
    private ElapsedTime presetTimer = new ElapsedTime();

    // Timer used for exiting early if needed
    ElapsedTime timer = new ElapsedTime();

    // Motor Powers
    final double WHEEL_SPEED_MAX = 1;
    final double INTAKE_SPEED_MAX = -1;
    final double SHOOTER_SPEED_MAX = -0.95;
    NormalizedRGBA colors;

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

    //Inital
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
    private DcMotor shooter;
    private Servo revolver;
    private Servo lift;
    private NormalizedColorSensor colorSensor;

    double kP = 0.0022; // bigger the error the faster we will fix it
    double kI = 0.00013; // provides extra boost when you get close to the target
    double kD = 0.00015; // dampens overshoot

    private double integralSum = 0;
    double lastError = 0;
    double targetDuration = 0;

    private int currentTargetIndex = 0; // Keeps track of the current target

    @Override
    public void loop() {
        if (currentTargetIndex < targets.length) {
            RobotTarget target = targets[currentTargetIndex];
            this.targetDuration = target.time;
            requestedState = target.state;

            if (timer.seconds() < targetDuration) {
                moveRobotTo(target.x, target.y, target.heading);
                stateMachine();
                moveRevolver();
                moveShooter();
                moveIntake();
                moveLift();
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
    }

    @Override
    public void start(){
        timer = new ElapsedTime();
        presetTimer = new ElapsedTime();
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
        lift = hardwareMap.get(Servo.class, "lift");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

        this.colors = colorSensor.getNormalizedColors();

        for(int i = 0; i < 3; i++){
            intakeStorage[i]=1;
        }


        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry configuration
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Pose2D startingPosition = new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Current X", startingPosition.getX(DistanceUnit.MM));
        telemetry.addData("Current Y", startingPosition.getY(DistanceUnit.MM));
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Current Heding", startingPosition.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();
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
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double deltaHeading = targetHeading - currentHeading;

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
        // deltaY = deltaY;
        // deltaX = -deltaX;

        double xPower = deltaX * kP;
        double yPower = deltaY * kP;
        double turnPower = -deltaHeading;

        // Negative currentHeading due to rotating global power counterclockwise
        double cosAngle = Math.cos(-currentHeading);
        double sinAngle = Math.sin(-currentHeading);

        // Using inverse rotational matrix
        double localX = xPower * cosAngle + yPower * sinAngle;
        double localY = -xPower * sinAngle + yPower * cosAngle;

        // Calculating individual wheel speeds
        double frontLeft = (localX - localY + turnPower) * wheelSpeed;
        double frontRight = (localX - localY - turnPower) * wheelSpeed;
        double backLeft = (localX + localY + turnPower) * wheelSpeed;
        double backRight = (localX + localY - turnPower) * wheelSpeed;

        FLeft.setPower(frontLeft);
        FRight.setPower(frontRight);
        BLeft.setPower(backLeft);
        BRight.setPower(backRight);

        telemetry.addData("Turn Power: ", turnPower);
        telemetry.addData("CurrentX: ", currentX);
        telemetry.addData("CurrentY: ", currentY);
        telemetry.addData("Current Heading: ", currentHeading);
        telemetry.addData("DeltaX", deltaX);
        telemetry.addData("DeltaY", deltaY);
        telemetry.addData("DeltaHeading", deltaHeading);
        telemetry.update();
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

                intakeStorage[0] = colorSeen();

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


                intakeStorage[1] = colorSeen();



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

                intakeStorage[2] = colorSeen();


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

    public void moveShooter(){
        shooter.setPower(shooterSpeed);
    }

    public void moveLift() {
        lift.setPosition(liftTarget);
    }

    public int colorSeen(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

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
}
