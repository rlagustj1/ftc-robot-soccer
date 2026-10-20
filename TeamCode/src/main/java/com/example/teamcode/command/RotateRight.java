package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Rotates clockwise in place. Command-based replacement for r_r.java. */
@TeleOp(name = "cmd r_r", group = "cmd")
public class RotateRight extends DirectionOpMode {

    public RotateRight() {
        super(0, 0, -1); // strafe, forward, turn
    }
}
