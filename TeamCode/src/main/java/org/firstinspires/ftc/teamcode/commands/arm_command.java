package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm;

import java.util.function.IntSupplier;

public class arm_command extends CommandBase {
    private final arm a;
    private final IntSupplier a1, a2;

    public arm_command(arm a, IntSupplier a1, IntSupplier a2) {
        this.a = a;
        this.a1 = a1;
        this.a2 = a2;

        addRequirements(this.a);
    }

    @Override
    public void execute() {
        a.update();

        int x = a1.getAsInt();
        int y = a2.getAsInt();

        if (x > 1) {
            a.fArmUp();
        } else if (x < 1) {
            a.fArmDown();
        } else {
            a.fArmStop();
        }

        if (y > 1) {
            a.sArmUp();
        } else if (y < 1) {
            a.sArmDown();
        } else {
            a.sArmStop();
        }
    }
}
