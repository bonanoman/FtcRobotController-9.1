package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="debug", group="brion's opmodes!")
public class debug extends LinearOpMode {
    private DcMotor left_motor;
    private DcMotor right_motor;
    @Override
    public void runOpMode() {

        left_motor = hardwareMap.get(DcMotor.class, "hd1");
        right_motor = hardwareMap.get(DcMotor.class, "hd2");
        
        left_motor.setDirection(DcMotor.Direction.REVERSE);
        right_motor.setDirection(DcMotor.Direction.FORWARD);

        left_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        
        while (opModeIsActive()) {
            
            if (gamepad1.a) {

                left_motor.setPower(0.1);
                right_motor.setPower(0.1);

            } else if (gamepad1.y) {

                left_motor.setPower(-0.1);
                right_motor.setPower(-0.1);

            }
            
        }
    }
}
