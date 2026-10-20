package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Joystick drive. The left stick is read in POLAR form, so a small diagonal and
 * a large diagonal keep the exact same heading and differ only in speed.
 *
 * Never finishes, so it is meant to be the drivetrain default command.
 */
public class DefaultDriveCommand extends CommandBase {

    public static final double DEADZONE   = 0.06; // stick slop to ignore
    public static final double EXPO       = 2.0;  // 1.0 = linear, higher = softer centre
    public static final double SLOW_SCALE = 0.35;
    public static final double TURN_SCALE = 0.80; // rotation is twitchy at 1.0

    private final DriveSubsystem drive;
    private final DoubleSupplier stickX, stickY, stickTurn;
    private final BooleanSupplier slow;
    private final Telemetry telemetry;

    public DefaultDriveCommand(DriveSubsystem drive,
                               DoubleSupplier stickX, DoubleSupplier stickY,
                               DoubleSupplier stickTurn, BooleanSupplier slow,
                               Telemetry telemetry) {
        this.drive = drive;
        this.stickX = stickX;
        this.stickY = stickY;
        this.stickTurn = stickTurn;
        this.slow = slow;
        this.telemetry = telemetry;

        addRequirements(drive); // setDefaultCommand refuses a command without this
    }

    @Override
    public void execute() {
        double sx = stickX.getAsDouble();
        double sy = stickY.getAsDouble();

        // Polar split: the direction comes from the raw stick and is never
        // touched, only the magnitude goes through deadzone plus response curve.
        double r = Math.min(1.0, Math.hypot(sx, sy));
        double mag = shape(r);

        double ux = 0.0, uy = 0.0;
        if (r > 1e-9) {
            ux = sx / r;
            uy = sy / r;
        }

        double turn = shape(stickTurn.getAsDouble()) * TURN_SCALE;

        if (slow.getAsBoolean()) {
            mag *= SLOW_SCALE;
            turn *= SLOW_SCALE;
        }

        drive.driveTeleop(ux, uy, mag, turn);

        if (telemetry != null) {
            double[] p = drive.getPowers();
            telemetry.addData("heading", "%.0f deg", Math.toDegrees(Math.atan2(uy, ux)));
            telemetry.addData("stick / mag", "%.2f -> %.2f", r, mag);
            telemetry.addData("turn", "%.2f", turn);
            telemetry.addData("powers", "lf %.2f  rf %.2f  lb %.2f  rb %.2f",
                    p[0], p[1], p[2], p[3]);
            telemetry.update();
        }
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    /** Deadzone with no step at its edge, followed by an expo response curve. */
    private static double shape(double v) {
        double sign = Math.signum(v);
        double a = Math.abs(v);
        if (a < DEADZONE) return 0.0;
        a = (a - DEADZONE) / (1.0 - DEADZONE); // re-stretch to 0..1, no jump
        return sign * Math.pow(a, EXPO);
    }
}
