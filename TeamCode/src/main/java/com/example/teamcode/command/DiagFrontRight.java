package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Drives diagonally front-right. Command-based replacement for d_fr.java. */
@TeleOp(name = "cmd d_fr", group = "cmd")
public class DiagFrontRight extends DirectionOpMode {

    public DiagFrontRight() {
        super(1, 1, 0); // strafe, forward, turn
    }
}
