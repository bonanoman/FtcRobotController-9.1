package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm_command;
import org.firstinspires.ftc.teamcode.commands.drive_command;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

import java.util.HashMap;

@TeleOp(name="FGC 2024 BAHAMAS")
public class FGC2024_BS extends CommandOpMode {
    private drivetrain dt;
    private arm a;

    private GamepadEx gp1, gp2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        dt = new drivetrain(hardwareMap);
        a = new arm(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        SlewRateLimiter limit1 = new SlewRateLimiter(4);
        SlewRateLimiter limit2 = new SlewRateLimiter(4);

        dt.setDefaultCommand(new drive_command(
                dt,
                () -> limit1.calculate(gp1.getLeftY()),
                () -> limit2.calculate(gp1.getRightY()),
                () -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> getDPAD(gp1)
        ));

        a.setDefaultCommand(new arm_command(
                a,
                () -> {
            if (gp2.getButton(Button.LEFT_BUMPER)) return -1;
            if (gp2.getButton(Button.RIGHT_BUMPER)) return 1;
            if (gp2.getButton(Button.Y)) return 1;
            if (gp2.getButton(Button.A)) return -1;
            return 0;
            },
                () -> {
            if (gp2.getButton(Button.LEFT_BUMPER)) return -1;
            if (gp2.getButton(Button.RIGHT_BUMPER)) return 1;
            if (gp2.getButton(Button.DPAD_UP)) return 1;
            if (gp2.getButton(Button.DPAD_DOWN)) return -1;
            return 0;
        }));

        gp2.getGamepadButton(Button.X)
                .whenPressed(new InstantCommand(() -> a.setServoState(arm.servo_state.FORWARD)))
                .whenReleased(new InstantCommand(() -> a.setServoState(arm.servo_state.IDLE)));

        gp2.getGamepadButton(Button.B)
                .whenPressed(new InstantCommand(() -> a.setServoState(arm.servo_state.REVERSE)))
                .whenReleased(new InstantCommand(() -> a.setServoState(arm.servo_state.IDLE)));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        telemetry.addLine("<------------------------>");
        telemetry.addLine("GAMEPAD 1");
        telemetry.addLine("------------------------");
        telemetry.addData("LEFT JOYSTICK Y", gp1.getLeftY());
        telemetry.addData("RIGHT JOYSTICK Y", gp1.getRightY());
        telemetry.addLine("------------------------");
        telemetry.addData("X", gp1.getButton(Button.X));
        telemetry.addData("Y", gp1.getButton(Button.Y));
        telemetry.addData("A", gp1.getButton(Button.A));
        telemetry.addData("B", gp1.getButton(Button.B));
        telemetry.addLine("------------------------");
        telemetry.addData("DPAD UP", gp1.getButton(Button.DPAD_UP));
        telemetry.addData("DPAD DOWN", gp1.getButton(Button.DPAD_DOWN));
        telemetry.addData("DPAD LEFT", gp1.getButton(Button.DPAD_LEFT));
        telemetry.addData("DPAD RIGHT", gp1.getButton(Button.DPAD_RIGHT));
        telemetry.addLine("<------------------------>");
        telemetry.addLine("");
        telemetry.addLine("<------------------------>");
        telemetry.addLine("GAMEPAD 2");
        telemetry.addLine("------------------------");
        telemetry.addData("LEFT JOYSTICK Y", gp2.getLeftY());
        telemetry.addData("RIGHT JOYSTICK Y", gp2.getRightY());
        telemetry.addLine("------------------------");
        telemetry.addData("X", gp2.getButton(Button.X));
        telemetry.addData("Y", gp2.getButton(Button.Y));
        telemetry.addData("A", gp2.getButton(Button.A));
        telemetry.addData("B", gp2.getButton(Button.B));
        telemetry.addLine("------------------------");
        telemetry.addData("DPAD UP", gp2.getButton(Button.DPAD_UP));
        telemetry.addData("DPAD DOWN", gp2.getButton(Button.DPAD_DOWN));
        telemetry.addData("DPAD LEFT", gp2.getButton(Button.DPAD_LEFT));
        telemetry.addData("DPAD RIGHT", gp2.getButton(Button.DPAD_RIGHT));
        telemetry.addLine("<------------------------>");
        telemetry.addLine("HARDWARE");
        telemetry.addLine("<------------------------>");
        interpretTelemetryPacket(a.getTelemetryPacket());
        telemetry.addLine("<------------------------>");
        interpretTelemetryPacket(dt.getTelemetryPacket());
    }

    public Button getDPAD(GamepadEx gp) {
        if (gp == null) return null;

        Button[] dpad = {Button.DPAD_UP, Button.DPAD_DOWN, Button.DPAD_LEFT, Button.DPAD_RIGHT};
        Button b = null;

        for (Button x : dpad) b = (gp.getButton(x)) ? x : null;

        return b;
    }

    public void interpretTelemetryPacket(HashMap<String, Object> h) {
        h.forEach((k, v) -> {
            if (v == null) {
                telemetry.addLine(k);
            } else {
                telemetry.addData(k, v);
            }
        });
    }
}
