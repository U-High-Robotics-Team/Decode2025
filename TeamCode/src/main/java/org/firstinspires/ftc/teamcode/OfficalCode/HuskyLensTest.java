package org.firstinspires.ftc.teamcode; // Installing according package that links entire directory.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Installing respective libraries for operations.
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;

@TeleOp(name = "HuskyLens AprilTag Telemetry", group = "HuskyLens") // Creating a teleop subsidiary class for that function.
public class HuskyLensTest extends LinearOpMode { // Creating a public class infrastructure for coding.
    private HuskyLens huskyLens; // Initializing a private variable that can only be referenced.
    private static final double APRILTAG_REAL_WIDTH = 6.0; // Standard width of APRIL TAG
    private static final double FOCAL_LENGTH_PIXEL_CONSTANT = 600.0; // constant needs to be determined (pixel length)?

    @Override // Changing orientation of function to OpMode.
    public void runOpMode() { // New method is created to entail entirety of functionality.
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens"); // Mapping device to I2C Bus 0.
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION); // Choosing HuskyLens setting.
        telemetry.addData("Status", "HuskyLens Initialized. Waiting for Start."); // Telemetry status retrieval.
        telemetry.update(); // Updating interface.

        waitForStart(); // Waiting for input.

        while (opModeIsActive()) { // Creating a loop for analysis and checks.
            HuskyLens.Block[] blocks = huskyLens.blocks(); // Creating an array of learnt objects (in the form of April Tags).
            String detectedPattern = "None detected"; // Offset backup.

            if (blocks != null && blocks.length > 0) { // Condition that checks if there are detected April Tags.
                telemetry.addData("AprilTags Detected", blocks.length); // Updating telemetry.
                for (int i = 0; i < blocks.length; i++) { // Loop that checks array for stored April Tag objects.
                    HuskyLens.Block block = blocks[i]; // Iterating through all stored tags for clarification of information.
                    telemetry.addData("Tag " + i + " ID", block.id); // ID --> Color variation.
                    telemetry.addData("Tag " + i + " X", block.x); // Horizontal displacement (location on April Tag).
                    telemetry.addData("Tag " + i + " Y", block.y); // Vertical displacement (location on April Tag).
                    telemetry.addData("Tag " + i + " Width", block.width); // Width of April Tags.
                    telemetry.addData("Tag " + i + " Height", block.height); // Height of April Tags.

                    if (block.width > 0) { // Condition to calculate distance.
                        double distance = (APRILTAG_REAL_WIDTH * FOCAL_LENGTH_PIXEL_CONSTANT) / block.width; // Using metrics from before to extend focal point onto width (cm).
                        telemetry.addData("Tag " + i + " Distance (approx)", "%.2f cm", distance); // cm distance from robot to HuskyLens is calculated.
                    }

                    switch (block.id) { // Creating a more focused method towards stored objects.
                        case 1: // Initial case, where ID = 1.
                            detectedPattern = "GPP"; // IDK check if this pattern corresponds to GPP.
                            break; // Ending functionality for mitigation of overrun information.
                        case 2:
                            detectedPattern = "PGP"; // IDK check if this pattern corresponds to PGP.
                            break; // Ending functionality for mitigation of overrun information.
                        case 3:
                            detectedPattern = "PPG"; // IDK check if this pattern corresponds to PPG.
                            break; // Ending functionality for mitigation of overrun information.
                        default:
                            detectedPattern = "Unknown ID"; // Backup integration of a potential error.
                            break; // Ending functionality for mitigation of overrun information.
                    }
                }
            } else {
                telemetry.addData("AprilTags Detected", "None"); // Updating telemetry accordingly.
            }
        
            telemetry.addData("Obelisk Pattern", detectedPattern); // Updating telemetry accordingly.
            telemetry.update(); // Updating telemetry accordingly.
        }
    }
}
