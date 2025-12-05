package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class ColorSensorTest extends OpMode {
    public RevColorSensorV3 colorSensor;

    public int colorSensorVal;

    public boolean parkTime;

    @Override
    public void init() {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

/* Green: Purple: Empty:
* */
    @Override
    public void loop() {
        colorSensorVal = colorSensor.argb();
        if (colorSensorVal < 0) {
            if (!parkTime) {
                gamepad1.rumble(0.9, 0, 200);  // 200 mSec burst on left motor.
                parkTime = true;  // Hold off any more triggers
            }
        } else {
            parkTime = false;  // We can trigger again now.
        }

        telemetry.addData("Color Sensor Value:", colorSensorVal);
        telemetry.addData("Gamepad Rumbling", gamepad1.isRumbling());
        telemetry.update();
    }
}
