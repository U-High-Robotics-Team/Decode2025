package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name = "HuskyLens AprilTag Telemetry", group = "HuskyLens")
public class HuskyLensTest extends LinearOpMode {

    private HuskyLens huskyLens;


    private static final double APRILTAG_REAL_WIDTH = 6.0; // 

    private static final double FOCAL_LENGTH_PIXEL_CONSTANT = 600.0; 

    @Override
    public void runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        telemetry.addData("Status", "HuskyLens Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            HuskyLens.Block[] blocks = huskyLens.blocks();

            if (blocks != null && blocks.length > 0) {
                telemetry.addData("AprilTags Detected", blocks.length);
                for (int i = 0; i < blocks.length; i++) {
                    HuskyLens.Block block = blocks[i];

                    telemetry.addData("Tag " + i + " ID", block.id);
                    telemetry.addData("Tag " + i + " X", block.x);
                    telemetry.addData("Tag " + i + " Y", block.y);
                    telemetry.addData("Tag " + i + " Width", block.width);
                    telemetry.addData("Tag " + i + " Height", block.height);

                    if (block.width > 0) {
                        double distance = (APRILTAG_REAL_WIDTH * FOCAL_LENGTH_PIXEL_CONSTANT) / block.width;
                        telemetry.addData("Tag " + i + " Distance (approx)", "%.2f inches", distance);
                    }
                }
            } else {
                telemetry.addData("AprilTags Detected", "None");
            }
            telemetry.update();
        }
    }
}
