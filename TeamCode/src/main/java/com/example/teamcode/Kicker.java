package com.example.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Two kick arms on the Expansion Hub, both driven together by one button.
 *
 * Configure names must match exactly:
 *   Expansion Hub motor port 0 -> "kick_left"
 *   Expansion Hub motor port 1 -> "kick_right"
 * Encoder cables must be plugged in; the arms are position controlled.
 *
 * Put both arms in the CHARGED (pulled back) position before pressing INIT.
 * The encoders are zeroed there, so 0 = charged and kickTicks = kicked.
 *
 * Cycle (X button):
 *   READY  -X->  FIRING  ->  COOLDOWN (COOLDOWN_TIME, X ignored)  ->  FIRED  -X->  CHARGING  ->  READY
 *
 * Position moves use RUN_TO_POSITION so the hub firmware closes the loop. Our
 * own loop is far too slow to stop a fast motor on time, and the 6000 rpm
 * motor turns more than a full revolution in one 10 ms loop at full power.
 */
public class Kicker {

    /** Numbers that depend on the motor. Each robot has a different one. */
    public static class Profile {
        final String name;
        final double ticksPerRev;   // encoder ticks per turn of the ARM (motor ticks x external gear ratio)
        final double kickDegrees;   // how far the arm swings forward when firing
        final double firePower;     // power cap while kicking
        final double fireTimeout;   // s; give up if the arm has not got there by now
        final double chargePower;   // power cap while pulling back
        final double chargeTimeout; // s
        final double holdPower;     // power cap while holding a position
        final int    tolerance;     // ticks; "close enough" to the target

        Profile(String name, double ticksPerRev, double kickDegrees,
                double firePower, double fireTimeout,
                double chargePower, double chargeTimeout,
                double holdPower, int tolerance) {
            this.name = name;
            this.ticksPerRev = ticksPerRev;
            this.kickDegrees = kickDegrees;
            this.firePower = firePower;
            this.fireTimeout = fireTimeout;
            this.chargePower = chargePower;
            this.chargeTimeout = chargeTimeout;
            this.holdPower = holdPower;
            this.tolerance = tolerance;
        }
    }

    // goBILDA 5203 Yellow Jacket 1150 rpm (5.2:1). 145.1 ticks/rev, enough
    // torque to swing an arm directly. 120 deg = 48 ticks, plenty of resolution.
    public static final Profile ROBOT1_1150 = new Profile("robot1 1150rpm",
            145.1, 120,
            1.0, 0.30,
            0.40, 1.5,
            0.30, 3);

    // goBILDA 5203 6000 rpm (1:1, no gearbox). Only 28 ticks/rev (about 13 deg
    // per tick) and very little torque, so: tight tolerance, gentle charge and
    // hold, short timeout so a stalled motor is cut off quickly.
    // If there is an external gear/belt reduction, multiply 28 by that ratio.
    public static final Profile ROBOT2_6000 = new Profile("robot2 6000rpm",
            28, 120,
            1.0, 0.15,
            0.20, 1.5,
            0.15, 1);

    public static final double COOLDOWN_TIME = 2.0; // s after a kick before X can charge again

    enum State { READY, FIRING, COOLDOWN, FIRED, CHARGING, FAULT }

    private final DcMotorEx left, right;
    private final Profile p;
    private final int kickTicks;
    private final ElapsedTime timer = new ElapsedTime(); // reset on every state change

    private State state = State.READY;
    private boolean holding = false;
    private String fault = "";

    public Kicker(HardwareMap hardwareMap, Profile profile) {
        p = profile;
        left = hardwareMap.get(DcMotorEx.class, "kick_left");
        right = hardwareMap.get(DcMotorEx.class, "kick_right");

        // The arms are mirror images, so one motor has to run the other way.
        // If an arm swings backwards when X is pressed, flip its direction here.
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(DcMotorSimple.Direction.REVERSE);

        kickTicks = Math.max(1, (int) Math.round(p.kickDegrees / 360.0 * p.ticksPerRev));

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setTargetPosition(0);
            m.setTargetPositionTolerance(p.tolerance);
            m.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            m.setPower(0); // nothing moves during INIT
        }
    }

    /** Call once per loop. {@code xPressed} must be a rising edge, not the raw button. */
    public void update(boolean xPressed) {
        if (!holding && state == State.READY) {
            // First loop after START: start holding the charged position.
            setPower(p.holdPower);
            holding = true;
        }

        double t = timer.seconds();
        switch (state) {
            case READY:
                if (xPressed) {
                    moveTo(kickTicks, p.firePower);
                    next(State.FIRING);
                }
                break;
            case FIRING:
                if (reached(kickTicks)) {
                    setPower(p.holdPower);
                    next(State.COOLDOWN);
                } else if (t >= p.fireTimeout) {
                    if (Math.min(left.getCurrentPosition(), right.getCurrentPosition()) < kickTicks / 4) {
                        // Barely moved: unplugged encoder, wrong direction or jammed arm.
                        // Stop before the motor burns or spins the arm round.
                        fail("arm did not move - check encoder cable / direction / jam");
                    } else {
                        setPower(p.holdPower);
                        next(State.COOLDOWN);
                    }
                }
                break;
            case COOLDOWN:
                if (t >= COOLDOWN_TIME) next(State.FIRED);
                break;
            case FIRED:
                if (xPressed) {
                    moveTo(0, p.chargePower);
                    next(State.CHARGING);
                }
                break;
            case CHARGING:
                if (reached(0) || t >= p.chargeTimeout) {
                    setPower(p.holdPower);
                    next(State.READY);
                }
                break;
            case FAULT:
                break;
        }
    }

    public void stop() {
        setPower(0);
    }

    public void addTelemetry(Telemetry telemetry) {
        String status = state.toString();
        if (state == State.COOLDOWN) {
            status += String.format(" (%.1f s)", Math.max(0, COOLDOWN_TIME - timer.seconds()));
        } else if (state == State.READY) {
            status += " - X to kick";
        } else if (state == State.FIRED) {
            status += " - X to charge";
        } else if (state == State.FAULT) {
            status += " - " + fault;
        }
        telemetry.addData("kicker", "%s", status);
        telemetry.addData("kick pos", "L %d  R %d  (charged 0, kicked %d)  [%s]",
                left.getCurrentPosition(), right.getCurrentPosition(), kickTicks, p.name);
    }

    private void moveTo(int target, double power) {
        left.setTargetPosition(target);
        right.setTargetPosition(target);
        setPower(power);
    }

    private boolean reached(int target) {
        return Math.abs(left.getCurrentPosition() - target) <= p.tolerance
            && Math.abs(right.getCurrentPosition() - target) <= p.tolerance;
    }

    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    private void fail(String why) {
        stop();
        fault = why;
        next(State.FAULT);
    }

    private void next(State s) {
        state = s;
        timer.reset();
    }
}
