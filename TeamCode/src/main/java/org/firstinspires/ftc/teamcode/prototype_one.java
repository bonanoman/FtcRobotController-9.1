package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="prototype one", group="brion's opmodes!")
public class prototype_one extends OpMode {

    // hardware
    private DcMotorEx left_motor;
    private DcMotorEx right_motor;

    // buttons & arm set up class
    private Buttons buttons;
    private Arm arm;

    // constants
    private final float SCALE = 0.6f;

    private boolean raising = false;
    private boolean raised = false;

    private void setPower(float left, float right) {

        double trigger = gamepad1.right_trigger;
        double left_power = 0;
        double right_power = 0;

        if (left < 0) {
            left_power = left * SCALE - trigger;
        } else if (left > 0) {
            left_power = left * SCALE + trigger;
        }

        if (right < 0) {
            right_power = right * SCALE - trigger;
        } else if (right > 0) {
            right_power = right * SCALE + trigger;
        }

        left_motor.setPower(left_power);
        right_motor.setPower(right_power);
    }

    private void driveTank() {setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);}

    public void line() {telemetry.addLine("\n---------------------------------------------------------------------\n");}

    public float clamp(float val, float min, float max) {
        if (min > max) {
            float temp_max = max;
            max = min; min = temp_max; // swap variables
        }
        return Math.max(min, Math.min(max, val));
    }

    @Override
    public void init() {

        // set up telemetry
        telemetry.setMsTransmissionInterval(50);

        // set up motor
        left_motor = hardwareMap.get(DcMotorEx.class, "m1");
        right_motor = hardwareMap.get(DcMotorEx.class, "m2");

        left_motor.setDirection(DcMotorEx.Direction.FORWARD);
        right_motor.setDirection(DcMotorEx.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        right_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        left_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // set up buttons
        buttons = new Buttons();

        // set up arms
        arm = new Arm(this);
        arm.init();
        arm.spinning = false;
        arm.calibrating = false;

    }

    @Override
    public void start() {
        resetRuntime();
    }

    @Override
    public void loop() {

        boolean options_pressed = buttons.ifPressed(gamepad2.options);
        boolean share_pressed = buttons.ifPressed(gamepad2.share);
        boolean guide_pressed = buttons.ifPressed(gamepad2.guide);
        boolean arrow_key_driving;

        if (options_pressed) {
            arm.resetCalibration();
            arm.calibrating = true; // if you press down on the two joysticks then the calibration will start
        }

        if (arm.calibrating) {
            arm.calibrate();
            return;
        }

        if (!raised) {

            telemetry.addLine("PRESS HOME TO INITIATE.");

            if (guide_pressed) {
                raising = true;
            }

            if (raising) {
                raised = arm.raise();
                if (raised) {
                    raising = false;
                    arm.resetEncoder();
                }
            }

            return;

        }

        // arm
        telemetry.addData("left bumper", gamepad2.left_bumper);
        telemetry.addData("right bumper", gamepad2.right_bumper);

        boolean bumpers = false;

        if (gamepad2.right_bumper) {
            bumpers = true;
            arm.moveArmUp();
        } else if (gamepad2.left_bumper) {
            bumpers = true;
            arm.moveArmDown();
        }

        if (!bumpers) {
            if (gamepad2.cross) {
                arm.moveHDUp();
            } else if (gamepad2.triangle) {
                arm.moveHDDown();
            } else {
                arm.freezeHD();
            }
            if (gamepad2.dpad_down) {
                arm.moveMIDDLEUp();
            } else if (gamepad2.dpad_up) {
                arm.moveMIDDLEDown();
            } else {
                arm.freezeMIDDLE();
            }
        }

        double tolerance = 0.05;
        boolean using_joystick = (
                Math.abs(gamepad1.left_stick_x) > tolerance ||
                        Math.abs(gamepad1.left_stick_y) > tolerance ||
                        Math.abs(gamepad1.right_stick_x) > tolerance ||
                        Math.abs(gamepad1.right_stick_y) > tolerance
        );

        if (gamepad1.dpad_up) {
            arrow_key_driving = true;
            setPower(1f, 1f);
        } else if (gamepad1.dpad_down) {
            arrow_key_driving = true;
            setPower(-1f, -1f);
        } else if (gamepad1.dpad_left) {
            arrow_key_driving = true;
            setPower(-1f, 1f);
        } else if (gamepad1.dpad_right) {
            arrow_key_driving = true;
            setPower(1f, -1f);
        } else {
            arrow_key_driving = false;
            if (!using_joystick) setPower(0, 0);
        }

        // drive mode
        driveTank();

        if (gamepad2.circle) {
            arm.spin(Servo.Direction.FORWARD);
        } else if (gamepad2.square) {
            arm.spin(Servo.Direction.REVERSE);
        }

        arm.spinning = (gamepad2.square || gamepad2.circle);
        if (gamepad2.circle) {
            arm.spin(Servo.Direction.FORWARD);
        } else if (gamepad2.square) {
            arm.spin(Servo.Direction.REVERSE);
        } else {
            arm.spin(Servo.Direction.FORWARD);
        }

        // telemetry
        line(); // just makes a line of the "-" character.
        telemetry.addData("status", "active");
        line();
        telemetry.addData("left joystick", "(%3.2f, %3.2f)", gamepad1.left_stick_x, -gamepad1.left_stick_y);
        telemetry.addData("right joystick", "(%3.2f, %3.2f)", gamepad1.right_stick_x, -gamepad1.right_stick_y);
        telemetry.addData("right trigger", "%2.2f", gamepad1.right_trigger);
        line();
        telemetry.addData("left wheels power", "%2.1f", left_motor.getPower());
        telemetry.addData("right wheels power", "%2.1f", right_motor.getPower());
        arm.telemetry();

        telemetry.update();

        buttons.reset();
    }

}
