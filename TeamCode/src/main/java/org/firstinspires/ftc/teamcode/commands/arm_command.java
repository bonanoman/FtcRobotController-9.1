package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.arm;

public class arm_command extends CommandBase {
    private final arm a;

    public arm_command(arm a) {
        this.a = a;

        addRequirements(this.a);
    }

    @Override
    public void execute() {

    }
}
