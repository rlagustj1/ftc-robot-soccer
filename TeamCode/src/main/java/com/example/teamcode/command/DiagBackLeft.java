package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives diagonally back-left. Command-based replacement for d_bl.java. */
@TeleOp(name = "cmd d_bl", group = "cmd")
public class DiagBackLeft extends DirectionOpMode {

    public DiagBackLeft() {
        super(-1, -1, 0); // strafe, forward, turn
    }
}
