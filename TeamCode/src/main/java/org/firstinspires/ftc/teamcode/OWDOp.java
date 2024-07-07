package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="omni wheel drive opmode", group="brion's opmodes!")
public class OWDOp extends LinearOpMode {

    private DcMotor front_left;
    private DcMotor front_right;
    private DcMotor back_left;
    private DcMotor back_right;

    private final Buttons buttons = new Buttons();

    private final float SCALE = 0.5f;

    private void setPower(double left, double right){

        front_left.setPower(left * SCALE + triggerScale(left));
        back_left.setPower(right * SCALE + triggerScale(right));
        front_right.setPower(left * SCALE + triggerScale(left));
        back_right.setPower(right * SCALE + triggerScale(right));

    }

    private double triggerScale(double power){ // CONCEPT: if the speed is scaled down and you want to go faster press down right trigger to increase speed.
        double output = gamepad1.right_trigger * SCALE;

        if (power < 0){ // if it's negative you want it to approach -1
            output *= -1;
        } else if (power == 0) { // if it's 0 you don't want to add anything to it
            output = 0;
        }

        return output;
    }

    @Override
    public void runOpMode() {

        // defining motors
        front_left = hardwareMap.get(DcMotor.class, "motor1");
        front_right = hardwareMap.get(DcMotor.class, "motor2");
        back_left = hardwareMap.get(DcMotor.class, "motor3");
        back_right = hardwareMap.get(DcMotor.class, "motor4");

        // motor setup
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        front_left.setDirection(DcMotorSimple.Direction.FORWARD);
        back_left.setDirection(DcMotorSimple.Direction.FORWARD);
        front_right.setDirection(DcMotorSimple.Direction.REVERSE);
        back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set telemetry and runtime
        telemetry.setMsTransmissionInterval(500);
        resetRuntime();

        waitForStart();

        // loop
        while (opModeIsActive()){

            // joystick variables
            float stick_x = gamepad1.left_stick_x;
            float stick_y = -gamepad1.left_stick_y;
            float turn = gamepad1.right_stick_x; // * SCALE/2; // scaled down turning

            // power calculations -- PI = 180 DEGREES
            double theta = Math.atan2(stick_y, stick_x) - (Math.PI / 4); // subtract 45 degrees so that the theta inside and outside the triangle are the same
            double magnitude = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2));
            double power_x = magnitude * Math.sin(theta) - turn; // distance (magnitude of the force) between current joystick position and resting joystick position * sin(theta) -- stick_x / magnitude of the force previously mentioned
            double power_y = magnitude * Math.cos(theta) + turn;

            // normalization
            if (Math.abs(power_x) > 1 || Math.abs(power_y) > 1){

                double largest_power = Math.max(Math.abs(power_x), Math.abs(power_y));
                power_x /= largest_power;
                power_y /= largest_power;

            }

            // assign powers
            setPower(power_x, power_y);

            // telemetry
            telemetry.addData("left joystick", String.format(Locale.getDefault(), "(%1$f, %2$f)", gamepad1.left_stick_x, -gamepad1.left_stick_y));
            telemetry.addData("right joystick", String.format(Locale.getDefault(), "(%1$f, %2$f)", gamepad1.right_stick_x, -gamepad1.right_stick_y));
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("x wheels power", front_left.getPowerFloat()); // front left, back right
            telemetry.addData("y wheels power", front_right.getPowerFloat()); // front right, back left

        }

    }
}
