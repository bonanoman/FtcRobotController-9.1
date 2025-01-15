package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

class DrivetrainSubsystem {
    private final Motor left_motor, right_motor;
    private final DifferentialDrive d;
    public DrivetrainSubsystem(HardwareMap hardwareMap) {
        // INITIALIZE DRIVETRAIN MOTORS
        left_motor = new Motor(hardwareMap, "left motor name");
        right_motor = new Motor(hardwareMap, "right motor name");

        // Set the run modes for the motors.
        left_motor.setRunMode(Motor.RunMode.RawPower);
        right_motor.setRunMode(Motor.RunMode.RawPower);

        // Set the zero power behavior i.e. what happens when the motor's power is 0.
        left_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Reverse the right motor so both motors go the same direction when the power is set.
        left_motor.setInverted(false);
        right_motor.setInverted(true);

        d = new DifferentialDrive(left_motor, right_motor);
    }

    public void drive(double left, double right) {
        d.tankDrive(left, right);
    }
}

@TeleOp(name="Example OpMode")
public class ExampleOpMode extends CommandOpMode {
    // private Motor example_motor;
    private DrivetrainSubsystem dt;
    @Override
    public void initialize() {
        // Runs once when the INIT button is pressed on the driver hub.
        // Similar to init() in regular op mode.

        /*
        // EXAMPLE OF MOTOR INITIALIZATION
        example_motor = new Motor(hardwareMap, "example motor name");
        example_motor.setInverted(false);
        example_motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        example_motor.setRunMode(Motor.RunMode.RawPower);
        example_motor.setPositionCoefficient(0.01); // ALWAYS START WITH A SMALL VALUE.
        example_motor.setPositionTolerance(10);
        */

        dt = new DrivetrainSubsystem(hardwareMap);

    }

    @Override
    public void run() {
        // Runs continuously after you press the START button until the STOP button is pressed.
        // Similar to loop() in regular op mode.

        // DRIVE ROBOT
        /* Set the power of the left and right motors (which ranges from 0 to 1) to the left and
           right joysticks y-value (which also ranges from 0 to 1).
        */
        dt.drive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        // You MUST call CommandScheduler.getInstance().run() in this function in order for your
        // commands to be ran.
        CommandScheduler.getInstance().run();
    }
}
