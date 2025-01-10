package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class drive_command extends CommandBase {
    private final drivetrain dt;
    private final DoubleSupplier lj, rj, rt;
    private final Supplier<Button> dpad;

    public drive_command(drivetrain dt, DoubleSupplier lj, DoubleSupplier rj, DoubleSupplier rt, Supplier<Button> dpad) {
        this.dt = dt;
        this.lj = lj;
        this.rj = rj;
        this.rt = rt;
        this.dpad = dpad;

        addRequirements(this.dt);
    }

    @Override
    public void execute() {
        double l, r;
        double scale = 0.6;
        double t = rt.getAsDouble();
        Button d = dpad.get();

        switch(d) {
            case DPAD_UP: l = r = 1;
                break;
            case DPAD_DOWN: l = r = -1;
                break;
            case DPAD_LEFT: l = -1; r = 1;
                break;
            case DPAD_RIGHT: l = 1; r = -1;
                break;
            default: l = lj.getAsDouble(); r = rj.getAsDouble();
                break;
        }

        if (Math.abs(l) > 0 || Math.abs(r) > 0) {
            // normalize values (scale them so they are between 0 and 1)
            double m = Math.max(l, r);
            l /= m;
            r /= m;

            // scale values (we don't want to go full throttle constantly)
            l *= scale;
            r *= scale;

            // use the trigger to approach 1 (full throttle)
            l = l + (1 - l) * t;
            r = r + (1 - r) * t;
        }

        dt.drive(l, r);
    }
}







































