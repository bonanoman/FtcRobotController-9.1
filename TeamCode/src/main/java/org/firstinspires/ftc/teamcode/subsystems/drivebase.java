package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class drivebase extends SubsystemBase {
    private final Motor l, r;
    private final IMU imu;
    private final DifferentialDrive d;

    public drivebase(final HardwareMap hm) {
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
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        )));

        imu.resetYaw();
    }

    public void drive(double lj, double rj) {
        d.tankDrive(lj, rj);

        double p = imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
        // how far does the robot need to tilt to recognize it's on an incline.
        double tilt = 10;
        l.setZeroPowerBehavior((Math.abs(p) < tilt) ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
        r.setZeroPowerBehavior((Math.abs(p) < tilt) ? Motor.ZeroPowerBehavior.BRAKE : Motor.ZeroPowerBehavior.FLOAT);
    }
}
