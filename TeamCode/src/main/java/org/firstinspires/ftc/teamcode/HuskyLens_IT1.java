package org.firstinspires.ftc.robotcontroller.external.samples; // Referencing the parent package which this program will classify into.

// Necessary library imports for lens functionality in java.

import com.qualcomm.hardware.dfrobot.HuskyLens; 
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "Sensor: HuskyLens", group = "Sensor") // Creating a teleop reference.
@Disabled // Negating teleop functionality.

public class HuskyLensMetrics extends LinearOpMode { // Creating a class extension to generalized sequential system.

  private HuskyLens huskyLens; // Creating a new reference object for latter usage (only within this program).

  @Override // Priority granted to following algorithm.
  public void runOpMode() { // Initialization and main-loop process.
    
  huskyLens = hardwareMap.get(HuskyLens.class, "huskylens"); // Registering local name to the private object created earlier.

    huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION) // Selecting a fixed system of AI analysis.

    telemetry.update(); // Updating data-sector component of the command center (telemetry).

    waitForStart(); // If a designated command, the following will halt unless the "start" prompt is activated.

    HuskyLens.Block[] blocks = huskylens.blocks(); // Array representation of all scanned objects by the camera is being initialized.

    telemetry.addData("Samples Analyzed:", blocks.length); // Checking if the camera is actually scanning and referencing samples to objects.x1

  } // Ending runOpMode() method in the algorithm.
  
} // Ending initial class of the entire program.

  
  



