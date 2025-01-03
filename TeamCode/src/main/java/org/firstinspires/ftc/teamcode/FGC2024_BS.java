package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.arm_command;
import org.firstinspires.ftc.teamcode.commands.drive_command;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.drivebase;
import org.firstinspires.ftc.teamcode.util.SlewRateLimiter;

import java.util.HashMap;

@TeleOp(name="FGC 2024 BAHAMAS")
public class FGC2024_BS extends CommandOpMode {
    private drivebase db;
    private arm a;

    private GamepadEx gp1, gp2;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        db = new drivebase(hardwareMap);
        a = new arm(hardwareMap);

        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);

        SlewRateLimiter limit = new SlewRateLimiter(4);

        db.setDefaultCommand(new drive_command(
                db,
                () -> limit.calculate(gp1.getLeftY()),
                () -> limit.calculate(gp1.getRightY()),
                () -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> getDPAD(gp1)
        ));

        a.setDefaultCommand(new arm_command(a,
                () -> {
            if (gp2.getButton(GamepadKeys.Button.LEFT_BUMPER)) return -1;
            if (gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) return 1;
            if (gp2.getButton(GamepadKeys.Button.Y)) return 1;
            if (gp2.getButton(GamepadKeys.Button.A)) return -1;
            return 0;
            },
                () -> {
            if (gp2.getButton(GamepadKeys.Button.LEFT_BUMPER)) return -1;
            if (gp2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) return 1;
            if (gp2.getButton(GamepadKeys.Button.DPAD_UP)) return 1;
            if (gp2.getButton(GamepadKeys.Button.DPAD_DOWN)) return -1;
            return 0;
        }));

        gp2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> {
                    a.setServoState(arm.servo_state.FORWARD);
                })
                .whenReleased(() -> {
                    a.setServoState(arm.servo_state.IDLE);
                });

        gp2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> {
                    a.setServoState(arm.servo_state.FORWARD);
                })
                .whenReleased(() -> {
                    a.setServoState(arm.servo_state.IDLE);
                });
    }

    @Override
    public void run() {
        telemetry.addLine("GAMEPAD 1");
        telemetry.addData("LEFT JOYSTICK Y", gp1.getLeftY());
        telemetry.addData("RIGHT JOYSTICK Y", gp1.getRightY());
        telemetry.addData("X", gp1.getButton(GamepadKeys.Button.X));
        telemetry.addData("Y", gp1.getButton(GamepadKeys.Button.Y));
        telemetry.addData("A", gp1.getButton(GamepadKeys.Button.A));
        telemetry.addData("B", gp1.getButton(GamepadKeys.Button.B));
        telemetry.addData("DPAD UP", gp1.getButton(GamepadKeys.Button.DPAD_UP));
        telemetry.addData("Y", gp1.getButton(GamepadKeys.Button.Y));
        telemetry.addData("A", gp1.getButton(GamepadKeys.Button.A));
        telemetry.addData("B", gp1.getButton(GamepadKeys.Button.B));

        telemetry.addLine("");

        telemetry.addLine("HARDWARE");
        telemetry.addLine("------------------------");
        interpretT(a.t);
        interpretT(db.t);
        telemetry.update();

        CommandScheduler.getInstance().run();
    }

    public int getDPAD(GamepadEx gp1) {
        if (gp1 == null) return -1;

        boolean[] b = {
                gp1.getButton(GamepadKeys.Button.DPAD_UP),
                gp1.getButton(GamepadKeys.Button.DPAD_DOWN),
                gp1.getButton(GamepadKeys.Button.DPAD_LEFT),
                gp1.getButton(GamepadKeys.Button.DPAD_RIGHT)
        };

        if (b[0]) return 0;
        if (b[1]) return 180;
        if (b[2]) return 270;
        if (b[3]) return 90;

        return -1;

    }

    public void interpretT(HashMap<String, Object> h) {
        h.forEach((k, v) -> {
            if (v == null) {
                telemetry.addLine(k);
            } else {
                telemetry.addData(k, v);
            }
        });
    }
}
