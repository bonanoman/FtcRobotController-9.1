package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="prototype one", group="brion's opmodes!")
public class prototype_one extends OpMode {

    // hardware
    private DcMotor left_motor;
    private DcMotor right_motor;
    private DistanceSensor distance_sensor;

    // buttons & arm set up class
    private Buttons buttons;
    private Arm arm;

    // toggles
    private boolean holding = false;

    // constants
    private final float SCALE = 0.5f;
    private final float MINIMUM_DISTANCE = 6;
    private final float PADDED_DISTANCE = 6;

    private void setPower(float left, float right) {

        double trigger = gamepad1.right_trigger * SCALE;
        double left_power;
        double right_power;

        if (left < 0) {
            left_power = left * SCALE - trigger + range_deceleration();
        } else if (left > 0) {
            left_power = left * SCALE + trigger - range_deceleration();
        } else {
            left_power = 0;
        }

        if (right < 0) {
            right_power = right * SCALE - trigger + range_deceleration();
        } else if (right > 0) {
            right_power = right * SCALE + trigger - range_deceleration();
        } else {
            right_power = 0;
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
        //left_motor = hardwareMap.get(DcMotor.class, "motor1");
        //right_motor = hardwareMap.get(DcMotor.class, "motor2");
        /*
        left_motor.setDirection(DcMotor.Direction.FORWARD);
        right_motor.setDirection(DcMotor.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        */
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
        arm.freezeHD();
        arm.freezeCORE();
    }

    @Override
    public void loop() {

        boolean x_pressed = buttons.ifPressed(gamepad1.x);
        boolean y_pressed = buttons.ifPressed(gamepad1.y);
        boolean a_pressed = buttons.ifPressed(gamepad1.a);
        boolean b_pressed = buttons.ifPressed(gamepad1.b);
        boolean options_pressed = buttons.ifPressed(gamepad1.options);
        boolean share_pressed = buttons.ifPressed(gamepad1.share);

        if (options_pressed) {
            arm.resetCalibration();
            arm.calibrating = true; // if you press down on the two joysticks then the calibration will start
        }

        if (arm.calibrating) {
            arm.calibrate();
            return;
        }
        if (share_pressed) arm.resetEncoder();
        if (x_pressed) holding = !holding;
        if (b_pressed) arm.spinning = !arm.spinning;

        // arm
        if (gamepad1.a) {
            arm.moveHDUp();
        } else if (gamepad1.y) {
            arm.moveHDDown();
        } else {
            arm.freezeHD();
        }

        if (gamepad1.dpad_down) {
            arm.moveCOREUp();
        } else if (gamepad1.dpad_up) {
            arm.moveCOREDown();
        } else {
            arm.freezeCORE();
        }

        arm.spin((holding) ? Servo.Direction.REVERSE : Servo.Direction.FORWARD); // holding is true? reverse to hold it. false? forward to spit it out

        // drive mode
        //driveTank();

        // telemetry
        line(); // just makes a line of the "-" character.
        telemetry.addData("status", "active");
        line();
        telemetry.addData("left joystick", "(%3.2f, %3.2f)", gamepad1.left_stick_x, -gamepad1.left_stick_y);
        telemetry.addData("right joystick", "(%3.2f, %3.2f)", gamepad1.right_stick_x, -gamepad1.right_stick_y);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        line();
        //telemetry.addData("left wheels power", "%3.2f", left_motor.getPower());
        //telemetry.addData("right wheels power", "%3.2f", right_motor.getPower());
        arm.telemetry();

        telemetry.update();

        buttons.reset();

    }

    @Override
    public void stop() {

        setPower(0, 0);
        arm.relax();

    }

}
