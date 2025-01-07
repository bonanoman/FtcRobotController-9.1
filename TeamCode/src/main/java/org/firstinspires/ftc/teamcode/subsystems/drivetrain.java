package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;

public class drivetrain extends SubsystemBase {
    private final Motor l, r;
    private final IMU imu;
    private final DifferentialDrive d;
    public HashMap<String, Object> t = new HashMap<>();

    public drivetrain(final HardwareMap hm) {
        l = new Motor(hm, "m1");
        r = new Motor(hm, "m2");
        imu = hm.get(IMU.class, "imu");

        d = new DifferentialDrive(l, r);

        l.setInverted(false);
        r.setInverted(true);

        l.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        l.setRunMode(Motor.RunMode.RawPower);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        imu.resetYaw();
    }

    public void drive(double lj, double rj) {
        d.tankDrive(lj, rj);
        t.put("DRIVINGINGINSDGID", null);

        double p = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        // how far does the robot need to tilt to recognize it's on an incline.
        double tilt = 10;
        Motor.ZeroPowerBehavior behavior = (Math.abs(p) < tilt) ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;
        l.setZeroPowerBehavior(behavior);
        r.setZeroPowerBehavior(behavior);
        update();
    }

    public void update() {
        // telemetry
        t.put("DRIVETRAIN", null);
        t.put("----------------", null);
        t.put("", null);
        t.put("LEFT MOTOR", null);

        t.put("----------- ", null);
        t.put(" ", null);
        t.put("LEFT POWER", l.get());
        t.put("LEFT POSITION", l.getCurrentPosition());

        t.put("RIGHT MOTOR", null);

        t.put("-----------", null);
        t.put("  ", null);
        t.put("RIGHT POWER", r.get());
        t.put("RIGHT POSITION", r.getCurrentPosition());
    }
}
