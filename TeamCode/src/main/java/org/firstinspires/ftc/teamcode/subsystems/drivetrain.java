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
    private final double pitch;
    public HashMap<String, Object> T = new HashMap<>();

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
        r.setRunMode(Motor.RunMode.RawPower);

        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        )));

        imu.resetYaw();
        pitch = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    public void drive(double lj, double rj) {
        d.tankDrive(lj, rj);

        double p = Math.abs(pitch - imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        // how far does the robot need to tilt to recognize it's on an incline.
        double tilt = 10;
        Motor.ZeroPowerBehavior behavior = (p < tilt) ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT;
        l.setZeroPowerBehavior(behavior);
        r.setZeroPowerBehavior(behavior);
    }

    public void arcadeDrive(double ljy, double rjx) {
        d.arcadeDrive(ljy * 0.6, rjx * 0.8);
    }

    public HashMap<String, Object> getTelemetryPacket() {
        T.put("DRIVETRAIN", null);
        T.put("---------------", null);
        T.put("LEFT POWER", l.get());
        T.put("LEFT POSITION", l.getCurrentPosition());
        T.put("--------------- ", null);
        T.put("RIGHT POWER", r.get());
        T.put("RIGHT POSITION", r.getCurrentPosition());
        T.put("---------------  ", null);
        T.put("PITCH", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
        return T;
    }
}
