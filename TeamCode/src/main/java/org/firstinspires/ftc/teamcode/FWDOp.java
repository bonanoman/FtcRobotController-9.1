package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.Locale;

@TeleOp(name="four wheel drive opmode", group="brion's opmodes!")
public class FWDOp extends LinearOpMode {

    // creating hardware variables
    private DcMotor left_motor;
    private DcMotor right_motor;
    
    // button stuff
    private final Buttons buttons = new Buttons();
    private boolean tank_mode = true;

    // constants
    private final float SCALE = 0.5f; // scales the powers down.

    /*
    COUNTS PER MOTOR REVOLUTION = 28 * GEARBOX_RATIO eg. 28 * 20 = 560
    GEAR REDUCTION = DRIVEN GEAR TEETH / DRIVING GEAR TEETH eg. 26 / 40 = 0.65
    WHEEL DIAMETER - self explanatory, convert it to the unit of measurement you're using. eg. 90 mm --> inches = 90 / 25.4 = 3.54330709
    COUNTS PER INCH = (COUNTS PER MOTOR REVOLUTION * GEAR REDUCTION) / CIRCUMFERENCE OF WHEEL or WHEEL DIAMETER * PI
                    = (560 * 0.65) / (3.54330709 * PI) = 322.73232233
     */

    private final double WHEEL_DIAMETER = 90 / 25.4; // 90mm to inches = 90 mm / 25.4
    private final double COUNTS_PER_INCH = (560 * 0.65) / (WHEEL_DIAMETER * Math.PI);

    private void setPower(float left, float right){
        left_motor.setPower(left * SCALE + triggerScale(left));
        right_motor.setPower(right * SCALE + triggerScale(right));
        // left_motor.setPower(left * scale); // regular wheel function
        // right_motor.setPower(right * scale);
    }

    private void setPosition(double left, double right){
        left_motor.setTargetPosition((int) left);
        right_motor.setTargetPosition((int) right);
    }

    private void setMode(DcMotor.RunMode rm){left_motor.setMode(rm); right_motor.setMode(rm);}

    private float triggerScale(float power){ // CONCEPT: if the speed is scaled down and you want to go faster press down right trigger to increase speed.
        float output = gamepad1.right_trigger * SCALE;

        if (power < 0){ // if it's negative you want it to approach -1
            output *= -1;
        } else if (power == 0) { // if it's 0 you don't want to add anything to it
            output = 0;
        }

        return output;
    }

    private void driveSimple(){

        // define joystick values
        float left_stick_y = -gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x / 4; // scaled down to prevent power being 0 while turning

        // calculate wheel powers
        float power_left = left_stick_y + right_stick_x;
        float power_right = left_stick_y - right_stick_x;

        // normalization
        if (Math.abs(power_left) > 1 || Math.abs(power_right) > 1) {

            float largest_power = Math.max(Math.abs(power_left), Math.abs(power_right));
            power_left /= largest_power;
            power_right /= largest_power;

        }

        setPower(power_left, power_right);

    }

    private void driveTank(){setPower(-gamepad1.left_stick_y, -gamepad1.right_stick_y);}

    private void resetEncoder(){

        left_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void driveEncoder(double speed, double left_inches, double right_inches){

        if (opModeIsActive()){

            resetEncoder();

            double left_position = left_motor.getCurrentPosition() + (left_inches * COUNTS_PER_INCH);
            double right_position = right_motor.getCurrentPosition() + (right_inches * COUNTS_PER_INCH);

            setPosition(left_position, right_position);
            setMode(DcMotor.RunMode.RUN_TO_POSITION);
            setPower((float) Math.abs(speed / SCALE), (float) Math.abs(speed / SCALE));
            // ^ abs val because you set a negative distance to make it go backward.
            // ^^ divided by SCALE to bypass the scaling in the setPower() function

            while(opModeIsActive() && left_motor.isBusy() && right_motor.isBusy() && !gamepad1.right_bumper){ // RIGHT BUMPER IS THE FAILSAFE.

                telemetry.addData("state", "auto");
                telemetry.addData("runtime", getRuntime());
                telemetry.addData("right trigger", gamepad1.right_trigger);
                telemetry.addData("left wheels power", "%.2d", left_motor.getPower());
                telemetry.addData("right wheels power", "%.2d", right_motor.getPower());
                telemetry.addData("moving to", "(%.2d, %.2d)", left_position, right_position);
                telemetry.addData("now at", "%.2d, %.2d", left_motor.getCurrentPosition(), right_motor.getCurrentPosition());

                telemetry.update();

            }

            setPower(0f, 0f);
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);

        }

    }

    @Override
    public void runOpMode(){

        // defining hardware variables (add device names tomorrow)
        left_motor = hardwareMap.get(DcMotor.class, "motor1");
        right_motor = hardwareMap.get(DcMotor.class, "motor2");

        // motor setup
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set telemetry and runtime
        telemetry.setMsTransmissionInterval(50);
        resetRuntime();

        waitForStart();

        while (opModeIsActive()){

            // Buttons
            boolean options_pressed = buttons.ifPressed(gamepad1.options, true);
            boolean a_pressed = buttons.ifPressed(gamepad1.a, true);

            if (a_pressed){

                driveEncoder(0.5, 12, 12); // should move a foot forward

            }

            // update tank_mode variable
            if (options_pressed){
                tank_mode = !tank_mode;
            }

            // choose which driving method
            if (tank_mode){
                driveTank();
            } else {
                driveSimple();
            }

            // telemetry things
            telemetry.addData("state", "active");
            telemetry.addData("runtime", getRuntime());
            telemetry.addData("left joystick", String.format(Locale.getDefault(),"(%.2f, %.2f)", gamepad1.left_stick_x, -gamepad1.left_stick_y));
            telemetry.addData("right joystick", String.format(Locale.getDefault(), "(%.2f, %.2f)", gamepad1.right_stick_x, -gamepad1.right_stick_y));
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("left wheels power", "%.2d", left_motor.getPower());
            telemetry.addData("right wheels power", "%.2d", right_motor.getPower());

            telemetry.update();

            buttons.reset();

        }

    }
}
