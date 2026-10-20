package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.CommandOpMode;

/**
 * Shared body for the single-direction test OpModes. A subclass supplies only
 * the motion mix; everything else is identical.
 *
 * Unlike the old plain-SDK files, the robot moves on PLAY, not on INIT.
 */
public abstract class DirectionOpMode extends CommandOpMode {

    static final double POWER = 0.5; // -1.0 to 1.0

    private final double strafe, forward, turn;

    protected DirectionOpMode(double strafe, double forward, double turn) {
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
    }

    @Override
    public void initialize() {
        // CommandScheduler is a singleton. Drop whatever a previous OpMode left
        // behind BEFORE the subsystem registers itself with the new instance.
        reset();

        DriveSubsystem drive = new DriveSubsystem(hardwareMap);
        schedule(new DriveDirectionCommand(drive, strafe, forward, turn, POWER));

        telemetry.addData("motion", "strafe %.0f / forward %.0f / turn %.0f",
                strafe, forward, turn);
        telemetry.addLine("Press PLAY to move.");
        telemetry.update();
    }
}
