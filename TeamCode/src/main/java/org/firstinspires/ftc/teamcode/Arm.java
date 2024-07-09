package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;

public class Arm {

    // convert degrees to arm positions?
    // get 180 degree height and the ticks the robot is at at that position
    // lock it between two buttons one bringing first elbow up to 180 and one pulling it down to 0.
    // x is the 0 position y is the 180 position. press a to raise arm, (button here) to lower it.
    // clamp it?

    // got explained the prototype's arms
    // ONLY RESET ENCODERS ONCE.

    // hardware
    private prototype_one opmode;
    private DcMotor hd_left_motor;
    private DcMotor hd_right_motor;
    private DcMotor core_left_motor;
    private DcMotor core_right_motor;
    private Servo left_servo;
    private Servo right_servo;

    public final HashMap<String, Integer> calibrated_positions = new HashMap<>();
    /*

    POSITIONS (IN TICKS)
    --------------------
    FIRST_ELBOW_MIN - HD motors of arm down
    FIRST_ELBOW_MAX - HD motors of arm up
    SECOND_ELBOW_MIN - CORE motors of arm down
    SECOND_ELBOW_MAX - CORE motors of arm up
    
     */

    // freezing motors
    private boolean frozen_hd = true;
    private int frozen_hd_pos = 0;
    private boolean frozen_core = true;
    private int frozen_core_pos = 0;

    // toggles
    public boolean spinning = false;

    // constants
    private final float SERVO_POWER = 1000;
    private final float HD_MOTOR_POWER = 0.1f;
    private final float CORE_MOTOR_POWER = 0.1f;
    private final int SMOOTHNESS_OF_TURNING = 100; // moves (max - min) / SMOOTHNESS_OF_TURNING ticks for every frame that you are pressing the up button for.
    private int MOVE_HD_BY;
    private int MOVE_CORE_BY;

    public Arm(prototype_one opmode) {
        this.opmode = opmode;
    }

    private void resetEncoder() {

        hd_left_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        hd_right_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        core_left_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        core_right_motor.setMode(RunMode.STOP_AND_RESET_ENCODER);

        hd_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        hd_right_motor.setMode(RunMode.RUN_USING_ENCODER);
        core_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        core_right_motor.setMode(RunMode.RUN_USING_ENCODER);

    }

    public void init() {

        hd_left_motor = opmode.hardwareMap.get(DcMotor.class, "hd1");
        hd_right_motor = opmode.hardwareMap.get(DcMotor.class, "hd2");
        core_left_motor = opmode.hardwareMap.get(DcMotor.class, "core1");
        core_right_motor = opmode.hardwareMap.get(DcMotor.class, "core2");
        left_servo = opmode.hardwareMap.get(Servo.class, "servo1");
        right_servo = opmode.hardwareMap.get(Servo.class, "servo2");

        hd_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        hd_right_motor.setMode(RunMode.RUN_USING_ENCODER);
        core_left_motor.setMode(RunMode.RUN_USING_ENCODER);
        core_right_motor.setMode(RunMode.RUN_USING_ENCODER);
        
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

        calibrated_positions.put("FIRST_ELBOW_MIN", null);
        calibrated_positions.put("FIRST_ELBOW_MAX", null);
        calibrated_positions.put("SECOND_ELBOW_MIN", null);
        calibrated_positions.put("SECOND_ELBOW_MAX", null);

        if (calibrated_positions.get("FIRST_ELBOW_MIN") == null) {

            try {
                calibrate();
            } catch (InterruptedException e) {
                opmode.telemetry.addLine("> CALIBRATION FAILED");
            }

        }

        MOVE_HD_BY = Math.abs(calibrated_positions.get("FIRST_ELBOW_MAX") - calibrated_positions.get("FIRST_ELBOW_MIN") / SMOOTHNESS_OF_TURNING);
        MOVE_CORE_BY = Math.abs(calibrated_positions.get("SECOND_ELBOW_MAX") - calibrated_positions.get("SECOND_ELBOW_MIN") / SMOOTHNESS_OF_TURNING);

    }

    public void calibrate() throws InterruptedException {

        while(!opmode.gamepad1.a) {
            opmode.telemetry.addLine("> FIRST ELBOW MIN POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();
        }

        calibrated_positions.replace("FIRST_ELBOW_MIN", hd_left_motor.getCurrentPosition());
        opmode.sleep(100);

        while (!opmode.gamepad1.a) {
            opmode.telemetry.addLine("> FIRST ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();
        }

        calibrated_positions.replace("FIRST_ELBOW_MAX", hd_left_motor.getCurrentPosition());
        opmode.sleep(100);

        while(!opmode.gamepad1.a) {
            opmode.telemetry.addLine("> SECOND ELBOW MIN POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();
        }

        calibrated_positions.replace("SECOND_ELBOW_MIN", core_left_motor.getCurrentPosition());
        opmode.sleep(100);

        while (!opmode.gamepad1.a) {
            opmode.telemetry.addLine("> SECOND ELBOW MAX POSITION (PRESS A WHEN SET)");
            opmode.telemetry.update();
        }

        calibrated_positions.replace("SECOND_ELBOW_MAX", core_left_motor.getCurrentPosition());
        opmode.sleep(100);

    }

    private boolean normalizeHD() {

        boolean fix = false;
        int fix_position = 0;

        if (hd_left_motor.getCurrentPosition() > calibrated_positions.get("FIRST_ELBOW_MAX")) {
            fix_position = calibrated_positions.get("FIRST_ELBOW_MAX");
            fix = true;
        }

        if (hd_left_motor.getCurrentPosition() < calibrated_positions.get("FIRST_ELBOW_MIN")) {
            fix_position = calibrated_positions.get("FIRST_ELBOW_MIN");
            fix = true;
        }

        if (fix) {
            moveHDTo(1, fix_position);
            return true;
        }

        return fix;

    }

    private boolean normalizeCORE() {

        boolean fix = false;
        int fix_position = 0;

        if (core_left_motor.getCurrentPosition() > calibrated_positions.get("FIRST_ELBOW_MAX")) {
            fix_position = calibrated_positions.get("FIRST_ELBOW_MAX");
            fix = true;
        }

        if (core_left_motor.getCurrentPosition() < calibrated_positions.get("FIRST_ELBOW_MIN")) {
            fix_position = calibrated_positions.get("FIRST_ELBOW_MIN");
            fix = true;
        }

        if (fix) {
            moveCORETo(1, fix_position);
            return true;
        }

        return fix;

    }

    private void moveHDTo(float power, int pos) {

        hd_left_motor.setMode(RunMode.RUN_TO_POSITION);
        hd_right_motor.setMode(RunMode.RUN_TO_POSITION);

        hd_left_motor.setTargetPosition(pos);
        hd_right_motor.setTargetPosition(pos);

        hd_left_motor.setPower(power);
        hd_right_motor.setPower(power);

    }

    private void moveCORETo(float power, int pos) {

        core_left_motor.setMode(RunMode.RUN_TO_POSITION);
        core_right_motor.setMode(RunMode.RUN_TO_POSITION);

        core_left_motor.setTargetPosition(pos);
        core_right_motor.setTargetPosition(pos);

        core_left_motor.setPower(power);
        core_right_motor.setPower(power);

    }

    public void moveHDMotorsUp() {

        if (normalizeHD() // if the motors are greater than max or less than min
                || hd_left_motor.getCurrentPosition() + MOVE_HD_BY > calibrated_positions.get("FIRST_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || hd_left_motor.getCurrentPosition() - MOVE_HD_BY < calibrated_positions.get("FIRST_ELBOW_MIN"))
            return; // don't do anything just return

        moveHDTo(HD_MOTOR_POWER, hd_left_motor.getCurrentPosition() + MOVE_HD_BY);

    }

    public void moveHDMotorsDown() {

        if (normalizeHD() // if the motors are greater than max or less than min
                || hd_left_motor.getCurrentPosition() + MOVE_HD_BY > calibrated_positions.get("FIRST_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || hd_left_motor.getCurrentPosition() - MOVE_HD_BY < calibrated_positions.get("FIRST_ELBOW_MIN"))
            return; // don't do anything just return

        moveHDTo(HD_MOTOR_POWER, hd_left_motor.getCurrentPosition() - MOVE_HD_BY);

    }

    public void moveCOREMotorsUp() {

        if (normalizeCORE() // if the motors are greater than max or less than min
                || core_left_motor.getCurrentPosition() + MOVE_CORE_BY > calibrated_positions.get("SECOND_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || core_left_motor.getCurrentPosition() - MOVE_CORE_BY < calibrated_positions.get("SECOND_ELBOW_MIN"))
            return; // don't do anything just return

        moveCORETo(CORE_MOTOR_POWER, core_left_motor.getCurrentPosition() + MOVE_CORE_BY);

    }

    public void moveCOREMotorsDown() {

        if (normalizeCORE() // if the motors are greater than max or less than min
                || core_left_motor.getCurrentPosition() + MOVE_CORE_BY > calibrated_positions.get("SECOND_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || core_left_motor.getCurrentPosition() - MOVE_CORE_BY < calibrated_positions.get("SECOND_ELBOW_MIN"))
            return; // don't do anything just return

        moveCORETo(CORE_MOTOR_POWER, core_left_motor.getCurrentPosition() - MOVE_CORE_BY);

    }

    public void freezeHD() {

        if (normalizeHD() // if the motors are greater than max or less than min
                || hd_left_motor.getCurrentPosition() + MOVE_HD_BY > calibrated_positions.get("FIRST_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || hd_left_motor.getCurrentPosition() - MOVE_HD_BY < calibrated_positions.get("FIRST_ELBOW_MIN"))
            return; // don't do anything just return

        if (!frozen_hd) {

            frozen_hd = true;
            frozen_hd_pos = hd_left_motor.getCurrentPosition();

        }

        moveHDTo(1, frozen_hd_pos);

    }

    public void unfreezeHD() {
        frozen_hd = false;
    }

    public void freezeCORE() {

        if (normalizeCORE() // if the motors are greater than max or less than min
                || core_left_motor.getCurrentPosition() + MOVE_CORE_BY > calibrated_positions.get("SECOND_ELBOW_MAX") // or the expected positions are greater than max or less than min
                || core_left_motor.getCurrentPosition() - MOVE_CORE_BY < calibrated_positions.get("SECOND_ELBOW_MIN"))
            return; // don't do anything just return

        if (!frozen_core) {

            frozen_core = true;
            frozen_core_pos = core_left_motor.getCurrentPosition();

        }

        moveCORETo(1, frozen_core_pos);

    }

    public void unfreezeCORE() {
        frozen_core = false;
    }

    public void spin(Servo.Direction d) {

        if (!spinning) return;

        double pos = left_servo.getPosition() + ((d == Servo.Direction.FORWARD) ? SERVO_POWER : -SERVO_POWER); // direction = forward? add servo. reverse? subtract servo

        left_servo.setPosition(pos);
        right_servo.setPosition(pos);

    }

    public void relax() { // set everything to its min position (no power required position)
        
        moveHDTo(HD_MOTOR_POWER, calibrated_positions.get("FIRST_ELBOW_MIN"));
        moveCORETo(CORE_MOTOR_POWER, calibrated_positions.get("SECOND_ELBOW_MIN"));
        spinning = false;

    }

    public void telemetry() {

        opmode.telemetry.addData("hd motor power", hd_left_motor.getPower());
        opmode.telemetry.addData("core motor power", core_left_motor.getPower());
        opmode.line();
        opmode.telemetry.addData("hd motor position", hd_left_motor.getCurrentPosition());
        opmode.telemetry.addData("core motor position", core_left_motor.getCurrentPosition());
        opmode.line();
        opmode.telemetry.addData("spinning", spinning);
        opmode.telemetry.addData("servo position", left_servo.getPosition());

    }

}