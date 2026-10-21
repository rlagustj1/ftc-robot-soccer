package com.example.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Robot 2: same drive as mecanum_drive, kicker tuned for the 6000 rpm motors. */
@TeleOp(name = "mecanum_drive (robot2 6000)")
public class mecanum_drive_6000 extends mecanum_drive {

    @Override
    protected Kicker.Profile kickerProfile() {
        return Kicker.ROBOT2_6000;
    }
}
