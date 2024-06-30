package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.ArrayList;
import java.util.Locale;

@TeleOp(name="fourWheelDriveOpMode", group="brion's opmodes!")
public class FWDOp extends LinearOpMode {

    // creating hardware variables
    private DcMotor left_motor;
    private DcMotor right_motor;
    
    // button stuff
    private int b_incrementer = 0;
    private final ArrayList<Boolean> b_list = new ArrayList<>();

    private final float scale = 0.5f; // scales the powers down.
    private boolean tank_mode = false;
    
    private void setPower(float left, float right){
        
        left_motor.setPower((left * scale > 0) ? left * scale + trigger_scale() : left * scale - trigger_scale());
        right_motor.setPower((right * scale > 0) ? right * scale + trigger_scale() : right * scale - trigger_scale());
        
        /* regular motor function
        // 
        // left_motor.setPower(left * scale);
        // right_motor.setPower(right * scale);
        */
    }

    private void driveSimple(){

        // define joystick values
        float left_stick_y = -gamepad1.left_stick_y;
        float right_stick_x = gamepad1.right_stick_x;

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

    private void driveTank(){

        // wheel powers
        float power_left = -gamepad1.left_stick_y;
        float power_right = -gamepad1.right_stick_y;

        setPower(power_left, power_right);

    }

    private float trigger_scale(){ // CONCEPT: if the speed is scaled down and you want to go faster press down right trigger to increase speed.
        return (scale < 1) ? gamepad1.right_trigger * scale : 0;
    }

    // automatically updating arraylist of buttons
    boolean ifPressed(boolean button, boolean down){ // down = do you want to detect when the button is pressed down or up

        boolean output;

        if (b_list.size() == b_incrementer){ // if there isn't a value for button then make one.
            b_list.add(false);
        }

        boolean b_was = b_list.get(b_incrementer); // last value of the button

        output = (button != b_was && button == down); // if the button isn't in the same state it was and (if down is true) the button is now down then the button was pressed.

        b_list.set(b_incrementer, button);

        b_incrementer++;
        return output;

    }

    @Override
    public void runOpMode(){

        // defining hardware variables (add device names tomorrow)
        left_motor = hardwareMap.get(DcMotor.class, "motor1");
        right_motor = hardwareMap.get(DcMotor.class, "motor2");

        // motor setup
        left_motor.setDirection(DcMotorSimple.Direction.FORWARD);
        right_motor.setDirection(DcMotorSimple.Direction.REVERSE);

        left_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // changed zero power behavior to float to avoid skidding sound when turning
        right_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // set telemetry and runtime
        telemetry.setMsTransmissionInterval(500);
        resetRuntime();

        waitForStart();

        while (opModeIsActive()) {

            // telemetry
            telemetry.addData("state", "active");
            telemetry.addData("runtime", getRuntime());

            // buttons
            boolean options_pressed = ifPressed(gamepad1.options, true);

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
            telemetry.addData("left joystick", String.format(Locale.getDefault(), "(%1$f, %2$f)", gamepad1.left_stick_x, -gamepad1.left_stick_y));
            telemetry.addData("right joystick", String.format(Locale.getDefault(), "(%1$f, %2$f)", gamepad1.right_stick_x, -gamepad1.right_stick_y));
            telemetry.addData("left wheels power", left_motor.getPowerFloat());
            telemetry.addData("right wheels power", right_motor.getPowerFloat());

            telemetry.update();

            b_incrementer = 0;

        }

    }
}
