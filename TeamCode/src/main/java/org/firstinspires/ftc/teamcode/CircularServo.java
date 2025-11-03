package org.firstinspires.ftc.teamcode; // Main library that allows for easy access of other methodological programs in this folder.

// Necessary library imports for lens functionality in java.

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;  
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "RotationServo") // Creating an accessible group name that can characterize the program.
  
public class RotationServo extends LinearOpMode { // Initializing a program that is inclusive of other methods.

  private Servo = rotationServo; // Initializing a servo instance.

  @Override // Progressing program past a "LinearOpMode" extension.

  public void runOpMode(){ // Creating a method that contains the majority of sub-methods.

    rotationServo = hardwareMap.get(Servo.class, "???"); // Getting servo information from control center. \NAME REQUIRED/

    rotationServo.setPosition(0.0); // Auto-orientation to a stable position.
    
    // Updating control system to contain validating information.
    
    telemetry.addData("Status", "Initialized");
    telemetry.update(); 

    while (opModeIsActive()){
     //not fully developed\\ if gamepadA.isClicked() > 0.3){
                                telemetry.update();
                              else{
                                return null;
                              }
        
      
    
    
