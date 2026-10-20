package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives diagonally back-right. Command-based replacement for d_br.java. */
@TeleOp(name = "cmd d_br", group = "cmd")
public class DiagBackRight extends DirectionOpMode {

    public DiagBackRight() {
        super(1, -1, 0); // strafe, forward, turn
    }
}
