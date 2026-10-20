package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives forward. Command-based replacement for straight.java. */
@TeleOp(name = "cmd straight", group = "cmd")
public class Straight extends DirectionOpMode {

    public Straight() {
        super(0, 1, 0); // strafe, forward, turn
    }
}
