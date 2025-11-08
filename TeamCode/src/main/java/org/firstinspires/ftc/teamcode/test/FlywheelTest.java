package org.firstinspires.ftc.teamcode.test;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.List;

@Config
@TeleOp
public class FlywheelTest extends OpMode {
    public VoltageSensor voltageSensor;
    public MotorEx shooter;

    public DcMotorEx intake;

    public DcMotorEx transfer;
    public List<LynxModule> hubs;

    public PIDFController shooterController;
    public static double shooterp = .00007, shooteri = .0000001, shooterd = .0000001;

    public static double shooterf = .0006;
    //shooterf = .0006
    //shooterp = .00007
    //shooterd = .0000001
    //shooteri = 0.0000001

    public static double velocityErrorTolerance;
    public static double shooterVelocityTarget;

    public static double intakePower, transferPower;

    public double shooterpid;

    public int ticks = 0;



    @Override
    public void init() {
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        shooterController = new PIDFController(shooterp, shooteri, shooterd, shooterf);

        shooter = new MotorEx(hardwareMap, "shooter", Motor.GoBILDA.RPM_1150);
        shooter.setRunMode(MotorEx.RunMode.RawPower);

        intake = hardwareMap.get(DcMotorImplEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        double voltageMultiplier = 12 / voltageSensor.getVoltage();
        shooter.setPositionTolerance(velocityErrorTolerance);
        shooterController.setPIDF(shooterp, shooteri, shooterd, shooterf);
        shooterpid = shooterController.calculate(shooter.getVelocity(), shooterVelocityTarget) * voltageMultiplier;
        shooter.set(shooterpid);

        intake.setPower(intakePower);
        transfer.setPower(transferPower);
//        ticks++;
//        if (ticks > 500) {
//            ticks = 0;
//        }
        telemetry.addData("Shooter PID Value", shooterpid);
        telemetry.addData("Shooter Velocity", shooter.getVelocity());
        telemetry.addData("Shooter Velocity Target", shooterVelocityTarget);
        telemetry.addData("At Setpoint", shooterController.atSetPoint());
        telemetry.addData("Intake Power", intakePower);
        telemetry.addData("Voltage", voltageSensor.getVoltage());
//        telemetry.addData("Ticks: ", ticks);
        telemetry.update();
    }
}

