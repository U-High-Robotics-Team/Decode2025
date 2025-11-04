package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name=ColorSensor)
public class ColorSensor extends OpMode {
  private ColorSensor colorSensor;
}

@Override
public void init() {
  colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
}

@Override 
public void runOpMode() {
  waitForStart();
  while (opModeIsActive()) {
            telemetry.addData("Light Detected", ((OpticalDistanceSensor) test_color).getLightDetected());
            NormalizedRGBA colors = test_color.getNormalizedColors(); 
            telemetry.update();
  }
}
