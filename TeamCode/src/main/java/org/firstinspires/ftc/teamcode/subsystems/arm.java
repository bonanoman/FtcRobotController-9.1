package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class arm extends SubsystemBase {
    private final Motor l, r, m;
    private final CRServo s;

    private final int min_r1 = 350;
    private final int max_r1 = 1576;

    public arm(HardwareMap hm) {
        l = new Motor(hm, "hd1");
        r = new Motor(hm, "hd2");
        m = new Motor(hm, "hd3");
        s = new CRServo(hm, "crs");

        l.setRunMode(Motor.RunMode.PositionControl);
        r.setRunMode(Motor.RunMode.PositionControl);
        m.setRunMode(Motor.RunMode.PositionControl);

        l.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        l.setInverted(false);
        r.setInverted(true);
        m.setInverted(false);

        l.setTargetPosition(min_r1);
        r.setTargetPosition(min_r1);

        s.setInverted(false);
    }
}
