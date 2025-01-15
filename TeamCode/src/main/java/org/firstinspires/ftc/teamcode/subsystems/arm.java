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

    private final int MIN_E1 = 150;
    private final int MAX_E1 = 1700;
    private final int MIN_E2 = 0;
    private final int MAX_E2 = 1576;
    private final int STEP = 50;

    private final HashMap<String, Object> T = new HashMap<>();

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

        double tolerance = 15;
        l.setPositionTolerance(tolerance);
        r.setPositionTolerance(tolerance);
        m.setPositionTolerance(tolerance);

        double kp = 0.05;
        l.setPositionCoefficient(kp);
        r.setPositionCoefficient(kp);
        m.setPositionCoefficient(kp);

        l.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        r.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        l.setInverted(false);
        r.setInverted(true);
        m.setInverted(false);
        s.setInverted(false);

        resetEncoders();

        l.setTargetPosition(MIN_E1);
        r.setTargetPosition(MIN_E1);
        m.setTargetPosition(MIN_E2);

        l.set(0);
        r.set(0);
        m.set(0);
    }

    public void setServoState(servo_state state) {
        this.s_state = state;
    }

    // first arm up
    public void fArmUp() {
        int goal = Range.clip(l.getCurrentPosition() + STEP, MIN_E1, MAX_E1);
        l.setTargetPosition(goal);
        r.setTargetPosition(goal);
    }

    // first arm down
    public void fArmDown() {
        int goal = Range.clip(l.getCurrentPosition() - STEP, MIN_E1, MAX_E1);
        l.setTargetPosition(goal);
        r.setTargetPosition(goal);
    }

    // first arm stop
    public void fArmStop() {
        int p = l.getCurrentPosition();
        l.setTargetPosition(p);
        r.setTargetPosition(p);
    }

    // second arm up
    public void sArmUp() {
        m.setTargetPosition(Range.clip(m.getCurrentPosition() + STEP, MIN_E2, MAX_E2));
    }

    // second arm down
    public void sArmDown() {
        m.setTargetPosition(Range.clip(m.getCurrentPosition() - STEP, MIN_E2, MAX_E2));
    }

    // second arm stop
    public void sArmStop() {
        m.setTargetPosition(m.getCurrentPosition());
    }

    @Override
    public void periodic() {
        double GLOBAL_MOTOR_POWER = 0.3;

        l.set(l.atTargetPosition() ? 0 : GLOBAL_MOTOR_POWER);
        r.set(r.atTargetPosition() ? 0 : GLOBAL_MOTOR_POWER);
        m.set(m.atTargetPosition() ? 0 : GLOBAL_MOTOR_POWER);
        s.set(s_state.POWER);
    }

    public HashMap<String, Object> getTelemetryPacket() {
        T.put("ARM", null);
        T.put("-------------------", null);
        T.put("LEFT POWER", l.get());
        T.put("LEFT POSITION", l.getCurrentPosition());
        T.put("------------------- ", null);
        T.put("RIGHT POWER", r.get());
        T.put("RIGHT POSITION", r.get());
        T.put("-------------------  ", null);
        T.put("SERVO STATE", this.s_state);
        T.put("SERVO POWER", s.get());
        return T;
    }
}
