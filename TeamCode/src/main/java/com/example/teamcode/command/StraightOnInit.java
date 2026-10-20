package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Command-based replacement for straight.java: the robot drives forward the
 * moment INIT is pressed, exactly like the old plain-SDK OpMode, and keeps
 * going through PLAY until STOP.
 *
 * The rest of the cmd package moves on PLAY instead ({@link Straight}); this
 * one exists for the bench test where you never want to touch PLAY.
 *
 * No hardware is touched here. {@link DriveSubsystem} owns the four motors
 * ("motor1".."motor4"), {@link DriveDirectionCommand} is the only thing that
 * commands them, and this OpMode just wires the two together.
 */
@TeleOp(name = "cmd straight (init)", group = "cmd")
public class StraightOnInit extends CommandOpMode {

    static final double POWER = 0.5; // -1.0 to 1.0

    private DriveSubsystem drive;
    private DriveDirectionCommand forward;

    @Override
    public void initialize() {
        // CommandScheduler is a singleton. Drop whatever a previous OpMode left
        // behind BEFORE the subsystem registers itself with the new instance.
        reset();

        // The one and only place the hardware map is read. SubsystemBase's
        // constructor registers the subsystem, so its periodic() runs too.
        drive = new DriveSubsystem(hardwareMap);

        forward = new DriveDirectionCommand(drive, 0, 1, 0, POWER); // strafe, forward, turn
        schedule(forward);

        telemetry.addData("power", POWER);
        telemetry.addLine("Moves on INIT. Press STOP to halt.");
        telemetry.update();
    }

    /**
     * CommandOpMode.runOpMode() parks in waitForStart() before it ever calls
     * run(), so a command scheduled in initialize() would not execute until
     * PLAY. Pumping the scheduler during the init phase as well is what makes
     * the wheels turn on INIT.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();

        while (opModeInInit()) {
            run();
            report();
        }

        waitForStart();

        while (!isStopRequested() && opModeIsActive()) {
            run();
            report();
        }

        // CommandScheduler.reset() only drops the singleton - it never calls
        // end() on what is still scheduled. Cancelling the command instead of
        // poking the subsystem keeps every motor write inside a command.
        CommandScheduler.getInstance().cancel(forward);
        reset();
    }

    /** Wheel powers come from the subsystem, not from the motors directly. */
    private void report() {
        double[] p = drive.getPowers(); // lf, rf, lb, rb
        telemetry.addData("lf / rf", "%.2f / %.2f", p[0], p[1]);
        telemetry.addData("lb / rb", "%.2f / %.2f", p[2], p[3]);
        telemetry.update();
    }
}
