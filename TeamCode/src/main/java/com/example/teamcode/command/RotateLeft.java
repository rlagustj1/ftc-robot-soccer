package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Rotates counter-clockwise. Command-based replacement for r_l.java. */
@TeleOp(name = "cmd r_l", group = "cmd")
public class RotateLeft extends DirectionOpMode {

    public RotateLeft() {
        super(0, 0, 1); // strafe, forward, turn
    }
}
