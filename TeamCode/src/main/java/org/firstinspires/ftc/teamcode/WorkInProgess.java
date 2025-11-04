package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="Global TeleOp Testing")
public class GlobalTeleOpTest extends OpMode {
    // Timer for Servos
    private final ElapsedTime timer = new ElapsedTime();
    final double WHEEL_SPEED_MAX = 1;
    final double WHEEL_SPEED_LIMITED = 0.17;

    double wheelSpeed = WHEEL_SPEED_MAX;

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    private DcMotor BLeft;
    private DcMotor BRight;
    private DcMotor FLeft;
    private DcMotor FRight;

    public void launchBall(){
        double xInitial = odo.getPosX();
        double theta = (double) Math.random(); // We don't know yet.... Fill in later.
        double yInitial = 0; // Fill in later from height of projector.
        final double initialVelocity = 10; // Inches/Second
        double tMax = (int) calculateHeight(xInitial, theta, yInitial, initialVelocity);
        double xProjected;
        double yProjected = 44;
      
        






    }


    public double calculateHeight(double offset, double theta, double yInitial, double iVelocity){
        double tMaxHeight = yInitial + (iVelocity 
        
        
    }

    public void moveRobot(){
        double forward = -gamepad1.left_stick_y * wheelSpeed; // (inverted Y-axis)
        double strafe = gamepad1.left_stick_x * wheelSpeed;
        double rotate = gamepad1.right_stick_x * wheelSpeed;

        if (gamepad1.right_trigger > 0.9){
            odo.resetPosAndIMU(); //resets the position to 0 and recalibrates the IMU
        }

        Pose2D pos = odo.getPosition();
        double heading =  pos.getHeading(AngleUnit.RADIANS);

        double cosAngle = Math.cos((Math.PI / 2)-heading);
        double sinAngle = Math.sin((Math.PI / 2)-heading);

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

    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");

        BLeft = hardwareMap.get(DcMotor.class, "backleft");
        BRight = hardwareMap.get(DcMotor.class, "backright");
        FLeft = hardwareMap.get(DcMotor.class, "frontleft");
        FRight = hardwareMap.get(DcMotor.class, "frontright");

        // reverse the motor directions
        BLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // odometry configuration
        odo.setOffsets(-84.0, -168.0); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        Pose2D startingPosition= new Pose2D(DistanceUnit.MM,-923.925, 1601.47, AngleUnit.RADIANS, 0);
        odo.setPosition(startingPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());
        telemetry.update();

        // Wait for the game to start (driver presses START)
        resetRuntime();
    }


    @Override
    public void loop() {
        moveRobot();
        odo.update();
    }
}
