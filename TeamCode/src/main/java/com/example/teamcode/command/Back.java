package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives backward. Command-based replacement for back.java. */
@TeleOp(name = "cmd back", group = "cmd")
public class Back extends DirectionOpMode {

    public Back() {
        super(0, -1, 0); // strafe, forward, turn
    }
}
