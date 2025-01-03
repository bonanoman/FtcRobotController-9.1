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

@TeleOp(name="FGC 2024 BAHAMAS")
public class FGC2024_BS extends CommandOpMode {
    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        drivebase db = new drivebase(hardwareMap);
        arm a = new arm(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        SlewRateLimiter limit = new SlewRateLimiter(4);

        db.setDefaultCommand(new drive_command(
                db,
                () -> limit.calculate(gp1.getLeftY()),
                () -> limit.calculate(gp1.getRightY()),
                () -> gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER),
                () -> getDPAD(gp1)
        ));

        a.setDefaultCommand(new arm_command(
            a
        ));

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
}
