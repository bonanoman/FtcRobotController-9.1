package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="prototype one", group="brion's opmodes!")
public class prototype_one extends OpMode {

    // hardware
    private DcMotorEx left_motor;
    private DcMotorEx right_motor;
    //private DistanceSensor distance_sensor;

    // buttons & arm set up class
    private Buttons buttons;
    private Arm arm;

    // constants
    private final float SCALE = 0.5f;
    private final float MINIMUM_DISTANCE = 6;
    private final float PADDED_DISTANCE = 6;

    private void setPower(float left, float right) {

        double trigger = gamepad1.right_trigger * SCALE;
        double left_power = 0;
        double right_power = 0;

        if (left < 0) {
            left_power = left * SCALE - trigger + range_deceleration();
        } else if (left > 0) {
            left_power = left * SCALE + trigger - range_deceleration();
        }

        if (right < 0) {
            right_power = right * SCALE - trigger + range_deceleration();
        } else if (right > 0) {
            right_power = right * SCALE + trigger - range_deceleration();
        }

        left_motor.setPower(left_power);
        right_motor.setPower(right_power);
    }

    private void driveTank() {setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);}

    private double range_deceleration() {
        /*
        double distance = distance_sensor.getDistance(DistanceUnit.INCH);
        double max = MINIMUM_DISTANCE + PADDED_DISTANCE;

        if (distance > max) {

            return 0;

        } else if (distance < MINIMUM_DISTANCE) {

            return 1;

        }

        return (max - distance) / PADDED_DISTANCE;
        */
        return 0; // TODO: temp
    }

    public void line() {telemetry.addLine("\n---------------------------------------------------------------------\n");}

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

        // set up distance sensor
        //distance_sensor = hardwareMap.get(DistanceSensor.class, "ds");

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
        boolean drive = true;
        boolean arrow_key_driving;

        if (options_pressed) {
            arm.resetCalibration();
            arm.calibrating = true; // if you press down on the two joysticks then the calibration will start
        }

        if (arm.calibrating) {
            arm.calibrate();
            return;
        }
        if (share_pressed) arm.resetEncoder();

        // arm
        if (gamepad2.right_bumper) {
            drive = false;
            arm.moveArmUp();
        } else if (gamepad2.left_bumper) {
            drive = false;
            arm.moveArmDown();
        }

        if (gamepad2.cross) {
            drive = false;
            arm.moveHDUp();
        } else if (gamepad2.triangle) {
            drive = false;
            arm.moveHDDown();
        } else {
            arm.freezeHD();
        }

        if (gamepad2.dpad_down) {
            drive = false;
            arm.moveMIDDLEUp();
        } else if (gamepad2.dpad_up) {
            drive = false;
            arm.moveMIDDLEDown();
        } else {
            arm.freezeMIDDLE();
        }

        if (gamepad1.dpad_up) {

            arrow_key_driving = true;
            drive = false;
            setPower(0.5f, 0.5f);

        } else if (gamepad1.dpad_down) {

            arrow_key_driving = true;
            drive = false;
            setPower(-0.5f, -0.5f);

        } else if (gamepad1.dpad_left) {

            arrow_key_driving = true;
            drive = false;
            setPower(-0.5f, 0.5f);

        } else if (gamepad1.dpad_right) {

            arrow_key_driving = true;
            drive = false;
            setPower(0.5f, -0.5f);

        } else {

            arrow_key_driving = false;
            setPower(0, 0);

        }

        arm.spinning = (gamepad2.square || gamepad2.circle);
        if (gamepad2.circle) {
            arm.spin(Servo.Direction.FORWARD);
        } else if (gamepad2.square) {
            arm.spin(Servo.Direction.REVERSE);
        }

        // drive mode
        if (drive) driveTank();
        int tolerance = 5;
        boolean using_joystick = (
                Math.abs(gamepad1.left_stick_x) > tolerance ||
                        Math.abs(gamepad1.left_stick_y) > tolerance ||
                        Math.abs(gamepad1.right_stick_x) > tolerance ||
                        Math.abs(gamepad1.right_stick_y) > tolerance
                );
        if (!drive && !arrow_key_driving && using_joystick) gamepad1.rumble(0.1, 0.1, Gamepad.RUMBLE_DURATION_CONTINUOUS);

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
