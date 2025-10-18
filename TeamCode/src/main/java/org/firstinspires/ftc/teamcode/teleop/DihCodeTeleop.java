package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.systems.Drive;
import org.firstinspires.ftc.teamcode.teleop.systems.Transport;
import org.firstinspires.ftc.teamcode.teleop.utils.GlobalVars;

import java.util.List;

@TeleOp
public class DihCodeTeleop extends OpMode {
    Drive drive;
    Transport transport;
    public List<LynxModule> allHubs;
    public LynxModule CtrlHub;

    public LynxModule ExpHub;


    @Override
    public void init() {
        GlobalVars.isAuto = false;
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
        drive.update(gamepad1, gamepad2);
        transport.update(gamepad1, gamepad2);
    }
}
