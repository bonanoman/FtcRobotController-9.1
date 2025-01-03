package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.drivebase;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class drive_command extends CommandBase {
    private final drivebase db;
    private final DoubleSupplier lj, rj, rt;
    private final IntSupplier dpad;

    public drive_command(drivebase db, DoubleSupplier lj, DoubleSupplier rj, DoubleSupplier rt, IntSupplier dpad) {
        this.db = db;
        this.lj = lj;
        this.rj = rj;
        this.rt = rt;
        this.dpad = dpad;

        addRequirements(this.db);
    }

    /** @noinspection UnusedAssignment*/
    @Override
    public void execute() {
        double l, r;
        double scale = 0.6;
        double t = rt.getAsDouble();
        int d = dpad.getAsInt();

        if (d == -1) {
            l = lj.getAsDouble() * scale;
            r = rj.getAsDouble() * scale;

            return;
        }
        switch (d) {
            case 0:
                l = 1;
                r = 1;
            case 180:
                l = -1;
                r = -1;
            case 270:
                l = -1;
                r = 1;
            case 90:
                l = 1;
                r = -1;
            default:
                l = 0;
                r = 0;


            l *= scale;
            r *= scale;

            l = l + (1 - l) * t;
            r = r + (1 - r) * t;
        }

        db.drive(l, r);
    }
}
