package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

import java.util.List;

@TeleOp(name = "DihCodeTeleop", group = "TeleOp")
public class DihCodeTeleop extends OpMode {
    Drive drive;
    Transport transport;
    public VoltageSensor voltageSensor;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;

    public double voltageMultiplier;


    @Override
    public void init() {
        voltageSensor =  hardwareMap.voltageSensor.iterator().next();
        GlobalVars.inAuto = false;
        drive = new Drive(hardwareMap, true, 0);
        transport = new Transport(hardwareMap);

        allHubs = hardwareMap.getAll(LynxModule.class);
        CtrlHub = allHubs.get(0);
        ExpHub = allHubs.get(1);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    @Override
    public void loop() {
        voltageMultiplier = 12 / voltageSensor.getVoltage();
        drive.update(gamepad1, gamepad2, voltageMultiplier);
        transport.update(gamepad1, gamepad2, voltageMultiplier);

        telemetry.addData("Flywheel Velocity", Transport.shooterVelocity);
        telemetry.addData("Flywheel Target Velocity", Transport.shooterVelocityTarget);
        telemetry.addData("Flywheel Fire Tolerance", Transport.fireTolerance);
        telemetry.addData("Flywheel Ready", transport.inRange(Transport.shooterVelocity, Transport.shooterVelocityTarget));
        telemetry.addData("Voltage", voltageSensor.getVoltage());
        telemetry.addData("Voltage Multiplier", voltageMultiplier);
        telemetry.addData("Intake Power", Transport.intake.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Transfer Power", Transport.transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FrontLeft Power", Drive.frontLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("FrontRight Power", Drive.frontRight.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BackLeft Power", Drive.backLeft.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("BackRight Power", Drive.backRight.getCurrent(CurrentUnit.AMPS));
        telemetry.update();
    }
}
