package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

public class Arm {

    // ONLY RESET ENCODERS ONCE.

    // hardware
    private OpMode opmode;
    private DcMotorEx hd_left_motor;
    private DcMotorEx hd_right_motor;
    private DcMotorEx hd_middle_motor;
    private CRServo spin_servo;

    private Integer min_middle_motor = 0;
    private Integer max_middle_motor = 1576;
    private Integer min_hd_motor = 0;
    private Integer max_hd_motor = -1234; // presets
    /*
    POSITIONS (IN TICKS)
    --------------------
    FIRST_ELBOW_MIN - HD motors down
    FIRST_ELBOW_MAX - HD motors up
    SECOND_ELBOW_MIN - MIDDLE motor down
    SECOND_ELBOW_MAX - MIDDLE motor up
     */

    // toggles
    public boolean spinning = false;
    public boolean calibrating = false;

    // constants
    private final float SPIN_SERVO_POWER = 1;
    private final float HD_MOTOR_POWER = 0.1f;
    private final float MIDDLE_MOTOR_POWER = 0.1f;
    private final Integer RAISED_POSITION = 350;

    private Integer MOVE_HD_BY = 70;
    private Integer MOVE_MIDDLE_BY = 70;

    private Integer virtual_hd_position = 0;
    private Integer virtual_middle_position = 0;

    private Integer freeze_hd = null;
    private Integer freeze_middle = null;

    public Arm(OpMode opmode) {
        this.opmode = opmode;
    }

    public void resetEncoder() {

        hd_left_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hd_right_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hd_middle_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        hd_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        hd_right_motor.setMode(RunMode.RUN_USING_ENCODER);
        hd_middle_motor.setMode(RunMode.RUN_USING_ENCODER);

    }

    public void init() {

        hd_left_motor = opmode.hardwareMap.get(DcMotorEx.class, "hd1");
        hd_right_motor = opmode.hardwareMap.get(DcMotorEx.class, "hd2");
        hd_middle_motor = opmode.hardwareMap.get(DcMotorEx.class, "hd3");
        spin_servo = opmode.hardwareMap.get(CRServo.class, "crs");

        hd_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        hd_right_motor.setMode(RunMode.RUN_USING_ENCODER);

        hd_left_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hd_right_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        hd_middle_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hd_left_motor.setDirection(DcMotorEx.Direction.REVERSE);
        hd_right_motor.setDirection(DcMotorEx.Direction.FORWARD);
        hd_middle_motor.setDirection(DcMotorEx.Direction.FORWARD);

        spin_servo.setDirection(DcMotorSimple.Direction.FORWARD);

        resetEncoder();

    }

    public void spin(Servo.Direction d) {

        if (!spinning) {
            spin_servo.setPower(0);
        } else {
            spin_servo.setPower((d == Servo.Direction.FORWARD) ? SPIN_SERVO_POWER : -SPIN_SERVO_POWER);
        }

    }

    public void moveHDTo(float power, Integer position) {

        hd_left_motor.setTargetPosition(position);
        hd_right_motor.setTargetPosition(position);
        hd_left_motor.setMode(RunMode.RUN_TO_POSITION);
        hd_right_motor.setMode(RunMode.RUN_TO_POSITION);
        virtual_hd_position = position;
        hd_left_motor.setPower(power);
        hd_right_motor.setPower(power);

    }

    public boolean raise() {

        boolean raised = false;
        int distance = Math.abs(hd_middle_motor.getCurrentPosition() - RAISED_POSITION);

        if (distance > 50) {

            moveMIDDLETo(MIDDLE_MOTOR_POWER, RAISED_POSITION);

        } else {

            raised = true;

        }

        return raised;

    }

    public Integer clamp(Integer val, Integer min, Integer max) {
        if (min > max) {
            Integer temp_max = max;
            max = min; min = temp_max; // swap variables
        }
        return Math.max(min, Math.min(max, val));
    }

    public void moveHDUp() {
        freeze_hd = null;
        Integer next_position = clamp(virtual_hd_position + MOVE_HD_BY, min_hd_motor, max_hd_motor);
        moveHDTo(HD_MOTOR_POWER, next_position);
    }

    public void moveHDDown() {
        freeze_hd = null;
        Integer next_position = clamp(virtual_hd_position - MOVE_HD_BY, min_hd_motor, max_hd_motor);
        moveHDTo(HD_MOTOR_POWER, next_position);
    }

    public void freezeHD() {
        if (freeze_hd == null) freeze_hd = hd_left_motor.getCurrentPosition();
        moveHDTo(1, freeze_hd);
    }

    public void moveMIDDLETo(float power, Integer position) {

        hd_middle_motor.setTargetPosition(position);
        hd_middle_motor.setMode(RunMode.RUN_TO_POSITION);
        virtual_middle_position = position;
        hd_middle_motor.setPower(power);

    }

    public void moveMIDDLEUp() {
        freeze_middle = null;
        Integer next_position = clamp(virtual_middle_position + MOVE_MIDDLE_BY, min_middle_motor, max_middle_motor);
        moveMIDDLETo(MIDDLE_MOTOR_POWER, next_position);
    }

    public void moveMIDDLEDown() {
        freeze_middle = null;
        Integer next_position = clamp(virtual_middle_position - MOVE_MIDDLE_BY, min_middle_motor, max_middle_motor);
        moveMIDDLETo(MIDDLE_MOTOR_POWER, next_position);
    }

    public void freezeMIDDLE() {
        if (freeze_middle == null) freeze_middle = hd_middle_motor.getCurrentPosition();
        moveMIDDLETo(1, freeze_middle);
    }

    public void moveArmUp() {

        freeze_hd = null;
        freeze_middle = null;
        Integer mid_next_position = clamp(virtual_middle_position - MOVE_MIDDLE_BY, min_middle_motor, max_middle_motor);
        Integer hd_next_position = clamp(virtual_hd_position + MOVE_HD_BY, min_hd_motor, max_hd_motor);
        moveMIDDLETo(0.2f, mid_next_position);
        moveHDTo(0.15f, hd_next_position);

    }

    public void moveArmDown() {

        freeze_hd = null;
        freeze_middle = null;
        Integer mid_next_position = clamp(virtual_middle_position + MOVE_MIDDLE_BY, min_middle_motor, max_middle_motor);
        Integer hd_next_position = clamp(virtual_hd_position - MOVE_HD_BY, min_hd_motor, max_hd_motor);
        moveMIDDLETo(0.2f, mid_next_position);
        moveHDTo(0.15f, hd_next_position);

    }

    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void resetCalibration() {
        min_hd_motor = null;
        max_hd_motor = null;
        max_middle_motor = null;
    }

    public void calibrate() {

        if (min_hd_motor == null) min_hd_motor = 0;
        if (min_middle_motor == null) min_middle_motor = 0;

        if (max_hd_motor == null) {

            if (opmode.gamepad2.dpad_up) {
                moveHDTo(HD_MOTOR_POWER, virtual_hd_position - MOVE_HD_BY);
                freeze_hd = null;
            } else if (opmode.gamepad2.dpad_down) {
                moveHDTo(HD_MOTOR_POWER, virtual_hd_position + MOVE_HD_BY);
                freeze_hd = null;
            } else {
                freezeHD();
            }

            calibrationTelemetry();
            opmode.telemetry.addLine("\n> FIRST ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();

            if (opmode.gamepad2.a) {
                max_hd_motor = hd_left_motor.getCurrentPosition();
            }

            if (max_hd_motor != null) sleep(250);

            return;

        } else if (max_middle_motor == null) {

            if (opmode.gamepad2.dpad_up) {
                moveMIDDLETo(MIDDLE_MOTOR_POWER, virtual_middle_position - MOVE_MIDDLE_BY);
                freeze_middle = null;
            } else if (opmode.gamepad2.dpad_down) {
                moveMIDDLETo(MIDDLE_MOTOR_POWER, virtual_middle_position + MOVE_MIDDLE_BY);
                freeze_middle = null;
            } else {
                freezeMIDDLE();
            }

            calibrationTelemetry();
            opmode.telemetry.addLine("\n> SECOND ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();

            if (opmode.gamepad2.a) {
                max_middle_motor = hd_middle_motor.getCurrentPosition();
            }

        }

        if (max_middle_motor != null) {
            calibrating = false;
        }

    }

    void line() {opmode.telemetry.addLine("\n---------------------------------------------------------------------\n");}

    private void calibrationTelemetry() {
        opmode.telemetry.addLine("*CALIBRATION SCREEN*");
        line();
        opmode.telemetry.addData("status", "calibrating");
        line();
        opmode.telemetry.addData("left hd motor position", hd_left_motor.getCurrentPosition());
        opmode.telemetry.addData("right hd motor position", hd_right_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("middle hd motor position", hd_middle_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("FIRST_ELBOW_MIN", (min_hd_motor == null) ? "uncalibrated" : min_hd_motor);
        opmode.telemetry.addData("FIRST_ELBOW_MAX", (max_hd_motor == null) ? "uncalibrated" : max_hd_motor);
        opmode.telemetry.addData("SECOND_ELBOW_MIN", (min_middle_motor == null) ? "uncalibrated" : min_middle_motor);
        opmode.telemetry.addData("SECOND_ELBOW_MAX", (max_middle_motor == null) ? "uncalibrated" : max_middle_motor);
        line();
    }

    public void telemetry() {
        line();
        opmode.telemetry.addData("hd motor power", "%2.1f", hd_left_motor.getPower());
        opmode.telemetry.addData("middle motor power", "%2.1f", hd_middle_motor.getPower());
        line();
        opmode.telemetry.addData("left hd motor position", hd_left_motor.getCurrentPosition());
        opmode.telemetry.addData("right hd motor position", hd_right_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("middle hd motor position", hd_middle_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("FIRST_ELBOW_MIN", (min_hd_motor == null) ? "uncalibrated" : min_hd_motor);
        opmode.telemetry.addData("FIRST_ELBOW_MAX", (max_hd_motor == null) ? "uncalibrated" : max_hd_motor);
        opmode.telemetry.addData("SECOND_ELBOW_MIN", (min_middle_motor == null) ? "uncalibrated" : min_middle_motor);
        opmode.telemetry.addData("SECOND_ELBOW_MAX", (max_middle_motor == null) ? "uncalibrated" : max_middle_motor);
        line();
        opmode.telemetry.addData("spinning", spinning);
        opmode.telemetry.addData("spinning servo power", spin_servo.getPower());
    }

}