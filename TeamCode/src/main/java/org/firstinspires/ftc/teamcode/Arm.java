package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Arm {

    // ONLY RESET ENCODERS ONCE.

    // hardware
    private OpMode opmode;
    private DcMotor hd_left_motor;
    private DcMotor hd_right_motor;
    private DcMotor core_left_motor;
    private DcMotor core_right_motor;
    private Servo left_servo;
    private Servo right_servo;

    private HashMap<String, Integer> calibrated_positions = new HashMap<>();
    /*
    POSITIONS (IN TICKS)
    --------------------
    FIRST_ELBOW_MIN - HD motors down
    FIRST_ELBOW_MAX - HD motors up
    SECOND_ELBOW_MIN - CORE motors down
    SECOND_ELBOW_MAX - CORE motors up
     */

    // toggles
    public boolean spinning = false;
    public boolean calibrating = false;

    // constants
    private final float SERVO_POWER = 1000;
    private final float HD_MOTOR_POWER = 0.1f;
    private final float CORE_MOTOR_POWER = 0.1f;

    private int MOVE_HD_BY = 50;
    private int MOVE_CORE_BY = 10;

    private int virtual_hd_position = 0;
    private int virtual_core_position = 0;

    public Arm(OpMode opmode) {
        this.opmode = opmode;
    }

    public void resetEncoder() {

        hd_left_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hd_right_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        core_left_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        core_right_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        hd_left_motor.setMode(RunMode.RUN_TO_POSITION);
        hd_right_motor.setMode(RunMode.RUN_TO_POSITION);
        core_left_motor.setMode(RunMode.RUN_TO_POSITION);
        core_right_motor.setMode(RunMode.RUN_TO_POSITION);

    }

    public void init() {

        hd_left_motor = opmode.hardwareMap.get(DcMotor.class, "hd1");
        hd_right_motor = opmode.hardwareMap.get(DcMotor.class, "hd2");
        core_left_motor = opmode.hardwareMap.get(DcMotor.class, "core1");
        core_right_motor = opmode.hardwareMap.get(DcMotor.class, "core2");
        left_servo = opmode.hardwareMap.get(Servo.class, "servo1");
        right_servo = opmode.hardwareMap.get(Servo.class, "servo2");

        hd_left_motor.setMode(RunMode.RUN_TO_POSITION);
        hd_right_motor.setMode(RunMode.RUN_TO_POSITION);
        core_left_motor.setMode(RunMode.RUN_TO_POSITION);
        core_right_motor.setMode(RunMode.RUN_TO_POSITION);

        hd_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hd_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        core_left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        core_right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hd_left_motor.setDirection(DcMotor.Direction.FORWARD);
        hd_right_motor.setDirection(DcMotor.Direction.REVERSE);
        core_left_motor.setDirection(DcMotor.Direction.FORWARD);
        core_right_motor.setDirection(DcMotor.Direction.REVERSE);
        left_servo.setDirection(Servo.Direction.FORWARD);
        right_servo.setDirection(Servo.Direction.REVERSE);

        resetEncoder();

        calibrated_positions.put("FIRST_ELBOW_MIN", 0);
        calibrated_positions.put("FIRST_ELBOW_MAX", 560);
        calibrated_positions.put("SECOND_ELBOW_MIN", 0);
        calibrated_positions.put("SECOND_ELBOW_MAX", 560);

    }

    public void spin(Servo.Direction d) {

        if (!spinning) return;

        double pos = left_servo.getPosition() + ((d == Servo.Direction.FORWARD) ? SERVO_POWER : -SERVO_POWER); // direction = forward? add servo. reverse? subtract servo

        left_servo.setPosition(pos);
        right_servo.setPosition(pos);

    }

    public void moveHDTo(float power, int position) {

        hd_left_motor.setTargetPosition(position);
        hd_right_motor.setTargetPosition(position);
        virtual_hd_position = position;
        hd_left_motor.setPower(power);
        hd_right_motor.setPower(power);

    }

    public void moveHDUp() {

        int next_position = virtual_hd_position + MOVE_HD_BY;
        int max = calibrated_positions.get("FIRST_ELBOW_MAX");
        boolean positive = Integer.signum(max) > 0;
        if (positive && next_position > max || !positive && next_position < max) next_position = max;

        moveHDTo(HD_MOTOR_POWER, next_position);

    }

    public void moveHDDown() {

        int next_position = virtual_hd_position - MOVE_HD_BY;
        int min = calibrated_positions.get("FIRST_ELBOW_MIN");
        boolean positive = Integer.signum(min) > 0;
        if (positive && next_position > min || !positive && next_position < min) next_position = min;

        moveHDTo(HD_MOTOR_POWER, next_position);

    }

    public void freezeHD() {

        moveHDTo(1, virtual_hd_position);

    }

    public void moveCORETo(float power, int position) {

        core_left_motor.setTargetPosition(position);
        core_right_motor.setTargetPosition(position);
        virtual_core_position = position;
        core_left_motor.setPower(power);
        core_right_motor.setPower(power);

    }

    public void moveCOREUp() {

        int next_position = virtual_core_position + MOVE_CORE_BY;
        int max = calibrated_positions.get("SECOND_ELBOW_MAX");
        boolean positive = Integer.signum(max) > 0;
        if (positive && next_position > max || !positive && next_position < max) next_position = max;

        moveCORETo(CORE_MOTOR_POWER, next_position);

    }

    public void moveCOREDown() {

        int next_position = virtual_core_position - MOVE_CORE_BY;
        int min = calibrated_positions.get("SECOND_ELBOW_MIN");
        boolean positive = Integer.signum(min) > 0;
        if (positive && next_position > min || !positive && next_position < min) next_position = min;

        moveCORETo(CORE_MOTOR_POWER, next_position);

    }

    public void freezeCORE() {

        moveCORETo(1, virtual_core_position);

    }

    private void sleep(long ms) {

        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

    }

    public void relax() { // set everything to its min position (no power required position)

        moveHDTo(HD_MOTOR_POWER, (calibrated_positions.get("FIRST_ELBOW_MIN") == null) ? 0 : calibrated_positions.get("FIRST_ELBOW_MIN"));
        moveCORETo(CORE_MOTOR_POWER, (calibrated_positions.get("SECOND_ELBOW_MIN") == null) ? 0 : calibrated_positions.get("SECOND_ELBOW_MIN"));
        sleep(500);
        spinning = false;

    }

    public void resetCalibration() {
        for (Map.Entry<String, Integer> e : calibrated_positions.entrySet()) calibrated_positions.replace(e.getKey(), null);
    }

    public void calibrate() {

        if (calibrated_positions.get("FIRST_ELBOW_MIN") == null) calibrated_positions.replace("FIRST_ELBOW_MIN", 0);
        if (calibrated_positions.get("SECOND_ELBOW_MIN") == null) calibrated_positions.replace("SECOND_ELBOW_MIN", 0);

        if (calibrated_positions.get("FIRST_ELBOW_MAX") == null) {

            if (opmode.gamepad1.dpad_up) {
                moveHDTo(HD_MOTOR_POWER, virtual_hd_position - MOVE_HD_BY);
            } else if (opmode.gamepad1.dpad_down) {
                moveHDTo(HD_MOTOR_POWER, virtual_hd_position + MOVE_HD_BY);
            } else {
                freezeHD();
            }

            calibrationTelemetry();
            opmode.telemetry.addLine("\n> FIRST ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();

            if (opmode.gamepad1.a) {
                calibrated_positions.replace("FIRST_ELBOW_MAX", hd_left_motor.getCurrentPosition());
            }

            if (calibrated_positions.get("FIRST_ELBOW_MAX") != null) sleep(250);

            return;

        } else if (calibrated_positions.get("SECOND_ELBOW_MAX") == null) {

            if (opmode.gamepad1.dpad_up) {
                moveCORETo(CORE_MOTOR_POWER, virtual_core_position - MOVE_CORE_BY);
            } else if (opmode.gamepad1.dpad_down) {
                moveCORETo(CORE_MOTOR_POWER, virtual_core_position + MOVE_CORE_BY);
            } else {
                freezeCORE();
            }

            calibrationTelemetry();
            opmode.telemetry.addLine("\n> SECOND ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();

            if (opmode.gamepad1.a) {
                calibrated_positions.replace("SECOND_ELBOW_MAX", core_left_motor.getCurrentPosition());
            }

        }

        if (calibrated_positions.get("SECOND_ELBOW_MAX") != null) {
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
        opmode.telemetry.addData("left core motor position", core_left_motor.getCurrentPosition());
        opmode.telemetry.addData("right core motor position", core_right_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("FIRST_ELBOW_MIN", (calibrated_positions.get("FIRST_ELBOW_MIN") == null) ? "uncalibrated" : calibrated_positions.get("FIRST_ELBOW_MIN"));
        opmode.telemetry.addData("FIRST_ELBOW_MAX", (calibrated_positions.get("FIRST_ELBOW_MAX") == null) ? "uncalibrated" : calibrated_positions.get("FIRST_ELBOW_MAX"));
        opmode.telemetry.addData("SECOND_ELBOW_MIN", (calibrated_positions.get("SECOND_ELBOW_MIN") == null) ? "uncalibrated" : calibrated_positions.get("SECOND_ELBOW_MIN"));
        opmode.telemetry.addData("SECOND_ELBOW_MAX", (calibrated_positions.get("SECOND_ELBOW_MAX") == null) ? "uncalibrated" : calibrated_positions.get("SECOND_ELBOW_MAX"));
        line();
    }

    public void telemetry() {

        line();
        opmode.telemetry.addData("hd motor power", hd_left_motor.getPower());
        opmode.telemetry.addData("core motor power", core_left_motor.getPower());
        line();
        opmode.telemetry.addData("left hd motor position", hd_left_motor.getCurrentPosition());
        opmode.telemetry.addData("right hd motor position", hd_right_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("left core motor position", core_left_motor.getCurrentPosition());
        opmode.telemetry.addData("right core motor position", core_right_motor.getCurrentPosition());
        line();
        opmode.telemetry.addData("FIRST_ELBOW_MIN", (calibrated_positions.get("FIRST_ELBOW_MIN") == null) ? "uncalibrated" : calibrated_positions.get("FIRST_ELBOW_MIN"));
        opmode.telemetry.addData("FIRST_ELBOW_MAX", (calibrated_positions.get("FIRST_ELBOW_MAX") == null) ? "uncalibrated" : calibrated_positions.get("FIRST_ELBOW_MAX"));
        opmode.telemetry.addData("SECOND_ELBOW_MIN", (calibrated_positions.get("SECOND_ELBOW_MIN") == null) ? "uncalibrated" : calibrated_positions.get("SECOND_ELBOW_MIN"));
        opmode.telemetry.addData("SECOND_ELBOW_MAX", (calibrated_positions.get("SECOND_ELBOW_MAX") == null) ? "uncalibrated" : calibrated_positions.get("SECOND_ELBOW_MAX"));

        line();
        opmode.telemetry.addData("spinning", spinning);
        //opmode.telemetry.addData("servo position", left_servo.getPosition());

    }

}