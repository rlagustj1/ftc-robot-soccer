package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives diagonally front-left. Command-based replacement for d_fl.java. */
@TeleOp(name = "cmd d_fl", group = "cmd")
public class DiagFrontLeft extends DirectionOpMode {

    public DiagFrontLeft() {
        super(-1, 1, 0); // strafe, forward, turn
    }
}
