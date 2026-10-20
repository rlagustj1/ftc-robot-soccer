package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Strafes left. Command-based replacement for left.java. */
@TeleOp(name = "cmd left", group = "cmd")
public class StrafeLeft extends DirectionOpMode {

    public StrafeLeft() {
        super(-1, 0, 0); // strafe, forward, turn
    }
}
