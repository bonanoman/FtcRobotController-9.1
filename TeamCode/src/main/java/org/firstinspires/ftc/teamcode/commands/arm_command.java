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

        int x = a1.getAsInt();
        int y = a2.getAsInt();

        switch (x) {
            case 1:
                a.fArmUp();
            case -1:
                a.fArmDown();
        }

        switch (y) {
            case 1:
                a.sArmUp();
            case -1:
                a.sArmDown();
        }

        a.update();
    }
}
