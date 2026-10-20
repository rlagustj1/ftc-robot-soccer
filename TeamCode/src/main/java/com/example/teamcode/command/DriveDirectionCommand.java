package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

/**
 * Drives one fixed direction until something else claims the drivetrain or the
 * OpMode stops. Command-based replacement for straight / back / left / right /
 * r_r / r_l / d_fr / d_fl / d_br / d_bl.
 */
public class DriveDirectionCommand extends CommandBase {

    private final DriveSubsystem drive;
    private final double strafe, forward, turn, power;

    public DriveDirectionCommand(DriveSubsystem drive,
                                 double strafe, double forward, double turn, double power) {
        this.drive = drive;
        this.strafe = strafe;
        this.forward = forward;
        this.turn = turn;
        this.power = power;

        addRequirements(drive); // nothing else may drive while this runs
    }

    @Override
    public void execute() {
        drive.driveScaled(strafe, forward, turn, power);
    }

    @Override
    public boolean isFinished() {
        return false; // runs until STOP, like the old while(opModeIsActive()) loop
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
