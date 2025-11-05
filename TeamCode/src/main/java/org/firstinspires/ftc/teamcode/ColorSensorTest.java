package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="ColorSensor")
public class ColorSensorTest extends OpMode {
    private NormalizedColorSensor colorSensor;



    @Override
    public void init() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");

    }

    @Override
    public void loop() {

        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        telemetry.addData("Red", "%.3f", colors.red);
        telemetry.addData("Green", "%.3f", colors.green);
        telemetry.addData("Blue", "%.3f", colors.blue);
        telemetry.update();
    }
}
