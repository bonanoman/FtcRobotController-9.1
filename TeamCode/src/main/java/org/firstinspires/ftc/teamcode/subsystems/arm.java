package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import java.util.HashMap;

public class arm extends SubsystemBase {
    private final Motor l, r, m;
    private final CRServo s;

    private final int min_r1 = 350;
    private final int max_r1 = 1700;
    private final int min_r2 = 0;
    private final int max_r2 = 1576;
    private final int step = 50;

    public HashMap<String, Object> t = new HashMap<String, Object>();

    private servo_state s_state = servo_state.IDLE;

    public enum servo_state {
        FORWARD(1),
        IDLE(0),
        REVERSE(-1);

        final double POWER;

        servo_state(double p) {
            this.POWER = p;
        }
    }

    public void resetEncoders() {
        l.resetEncoder();
        r.resetEncoder();
        m.resetEncoder();
    }

    public arm(HardwareMap hm) {
        l = new Motor(hm, "hd1");
        r = new Motor(hm, "hd2");
        m = new Motor(hm, "hd3");
        s = new CRServo(hm, "crs");

        l.setRunMode(Motor.RunMode.PositionControl);
        r.setRunMode(Motor.RunMode.PositionControl);
        m.setRunMode(Motor.RunMode.PositionControl);

        l.setPositionTolerance(15);
        r.setPositionTolerance(15);
        m.setPositionTolerance(15);

        double kp = 0;
        l.setPositionCoefficient(kp);
        r.setPositionCoefficient(kp);
        m.setPositionCoefficient(kp);

        l.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        l.setInverted(false);
        r.setInverted(true);
        m.setInverted(false);

        resetEncoders();

        l.setTargetPosition(min_r1);
        r.setTargetPosition(min_r1);
        m.setTargetPosition(min_r2);

        double mp = 0.3;
        l.set(mp);
        r.set(mp);
        m.set(mp);

        s.setInverted(false);
    }

    public void setServoState(servo_state state) {
        this.s_state = state;
    }

    public servo_state getServoState() {
        return this.s_state;
    }

    // first arm up
    public void fArmUp() {
        int goal = Range.clip(l.getCurrentPosition() + step, min_r1, max_r1);
        l.setTargetPosition(goal);
        r.setTargetPosition(goal);
    }

    // first arm down
    public void fArmDown() {
        int goal = Range.clip(l.getCurrentPosition() - step, min_r1, max_r1);
        l.setTargetPosition(goal);
        r.setTargetPosition(goal);
    }

    // second arm up
    public void sArmUp() {
        int goal = Range.clip(m.getCurrentPosition() + step, min_r2, max_r2);
        m.setTargetPosition(goal);
    }

    // second arm down
    public void sArmDown() {
        int goal = Range.clip(m.getCurrentPosition() - step, min_r2, max_r2);
        m.setTargetPosition(goal);
    }

    public void update() {
        s.set(s_state.POWER); // servo moves based on state

        //telemetry
        t.put("ARM ONE LEFT MOTOR", null);
        t.put("LEFT POWER", l.get());
        t.put("LEFT POSITION", l.getCurrentPosition());

        t.put("ARM ONE RIGHT MOTOR", null);
        t.put("RIGHT POWER", r.get());
        t.put("RIGHT POSITION", r.get());

        t.put("SERVO", null);
        t.put("STATE", this.s_state);
        t.put("POWER", s.get());

    }
}
