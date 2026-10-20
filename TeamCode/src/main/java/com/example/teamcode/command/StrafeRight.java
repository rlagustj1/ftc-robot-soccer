package com.example.teamcode.command;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Strafes right. Command-based replacement for right.java. */
@TeleOp(name = "cmd right", group = "cmd")
public class StrafeRight extends DirectionOpMode {

    public StrafeRight() {
        super(1, 0, 0); // strafe, forward, turn
    }
}
