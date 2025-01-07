package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class drive_command extends CommandBase {
    private final drivetrain dt;
    private final DoubleSupplier lj, rj, rt;
    private final IntSupplier dpad;

    public drive_command(drivetrain dt, DoubleSupplier lj, DoubleSupplier rj, DoubleSupplier rt, IntSupplier dpad) {
        this.dt = dt;
        this.lj = lj;
        this.rj = rj;
        this.rt = rt;
        this.dpad = dpad;

        addRequirements(this.dt);
    }

    @Override
    public void execute() {
        dt.update();

        double l, r;
        double scale = 0.6;
        double t = rt.getAsDouble();
        int d = dpad.getAsInt();

        if (d == -1) {
            l = lj.getAsDouble();
            r = rj.getAsDouble();
        } else if (d==0) {
            l=1; r=1;
        } else if (d==180) {
            l=-1; r=-1;
        } else if (d==90) {
            l=-1; r=1;
        } else if (d==270) {
            l=1; r=-1;
        } else {
            dt.update();
            return;
        }

        l *= scale;
        r *= scale;

        l = l + (1 - l) * t;
        r = r + (1 - r) * t;

        dt.drive(l, r);
    }
}
