package com.example.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * The kicker: two motors on ONE shared shaft, on the Expansion Hub.
 *
 * Configure names must match exactly:
 *   Expansion Hub motor port 0 -> "kick_left"
 *   Expansion Hub motor port 1 -> "kick_right"
 * Encoder cables must be plugged in. Check them with the "kicker diag" OpMode.
 *
 * Because the motors share a shaft they are one axis: same power to both,
 * position = average of the two encoders. If the encoders disagree by more
 * than MISMATCH_DEGREES the motors are fighting each other (one direction set
 * wrong, or a bad encoder), which stalls both and can brown out the robot, so
 * the kicker shuts down with a FAULT that X can not clear. Run kicker diag.
 *
 * Put the arm in the CHARGED (pulled back) position before pressing INIT.
 * The encoders are zeroed there, so 0 = charged and the kick angle is measured from it.
 *
 * Cycle (X button):
 *   READY  -X->  FIRING  ->  COOLDOWN (COOLDOWN_TIME, X ignored)  ->  FIRED  -X->  CHARGING  ->  READY
 *   FAULT  -X->  CHARGING  (pull the arm back and try again; not after a mismatch)
 *
 * How a kick works
 * ----------------
 * Earlier versions left the kick to the hub's RUN_TO_POSITION. That brakes in
 * proportion to the distance left, which starts far too late for a direct-drive
 * 1150 rpm arm (2700+ deg/s even at low power), and the code then dropped the
 * motor power the moment the arm passed the target, still at full speed. Every
 * setting slammed into the 180 deg shield. (Worse, the encoders count
 * against the motors, so RUN_TO_POSITION actually ran away: see ENCODER_SIGN.)
 *
 * Now the kick is driven open-loop in three phases:
 *
 *   ACCEL  raw power (soft/hard). Raw power, so take-off is as sharp as the motors allow.
 *   BRAKE  full reverse power, started when
 *            position + speed x latency + speed^2 / (2 x brakeDecel)  >=  target
 *          i.e. as soon as the arm could only just stop by the target. A faster
 *          arm starts braking earlier, so the stop angle does not depend on power.
 *   DONE   once the arm has stopped: 0 power, BRAKE.
 *
 * brakeDecel is how hard the motors can stop the arm (ticks/s^2). Too high and
 * the arm brakes too late and overshoots; too low and it brakes early and the
 * kick is soft. Each kick measures the real value and the next kick uses
 * LEARN_MARGIN of it (learnBrake), so it settles after a few kicks; an
 * overshoot past maxDegrees cuts it hard instead. Y scales it live.
 *
 * Positions and speeds come from one bulk read per loop, which keeps the loop
 * short; latency is the measured loop time.
 *
 * After the kick the arm is held where it stopped (COOLDOWN, FIRED), so
 * gravity can not drop it. Charging (X again) lowers it slowly: a target
 * slides down to 0 at lowerSpeedDeg and the arm follows it with
 *   power = trackKp x (target - position) + trackKd x (target speed - speed)
 * The lower power limit (lowerMaxDown) is small and the upper one bigger, so
 * the loop mostly holds the arm back against gravity instead of driving it
 * down; letting it fall slammed the robot. (All of this is our own loop, not
 * the hub's RUN_TO_POSITION: see ENCODER_SIGN.)
 * In READY and FAULT the motors get 0 power on BRAKE, so the arm can not
 * start moving unless X was pressed.
 */
public class Kicker {

    // The motors face each other on the shared shaft, so they run opposite ways.
    // Robot 1: with these values + power swings the arm UP (the kick).
    public static final DcMotorSimple.Direction LEFT_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction RIGHT_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // On robot 1 both encoders count AGAINST their motor: + power drives the arm
    // up while the encoders count down (seen in kicker diag with both direction
    // settings). setDirection flips power and encoder together, so it can not
    // fix this; every reading is multiplied by ENCODER_SIGN instead. It also
    // means the hub's own encoder modes (RUN_TO_POSITION, RUN_USING_ENCODER)
    // run away, which is what slammed the arm into the shield, moved it on its
    // own and browned out the robot. Only RUN_WITHOUT_ENCODER is used.
    // KickerDiag uses the same three values.
    public static final int ENCODER_SIGN = -1;

    /** Numbers that depend on the motor. Each robot has a different one. */
    public static class Profile {
        final String name;
        final double ticksPerRev;    // encoder ticks per turn of the ARM
        final double maxTicksPerSec; // free speed at the encoder
        final double kickDegrees;    // where the arm should stop
        final double maxDegrees;     // past this the arm always brakes, whatever the maths says
        final double softPower;      // raw power while kicking, SOFT setting (B toggles)
        final double hardPower;      // raw power while kicking, HARD setting
        final double brakeDecel;     // ticks/s^2 the motors can stop the arm with (see class comment)
        final double fireTimeout;    // s; the whole kick must be over by then
        final double lowerSpeedDeg;  // deg/s the arm is lowered at when charging
        final double lowerMaxDown;   // power limit pushing the arm down (small: gravity helps)
        final double lowerMaxUp;     // power limit holding the arm up against gravity
        final int    tolerance;      // ticks; "close enough" to 0 when charging
        final double trackKp;        // power per tick behind the target (hold / lower)
        final double trackKd;        // power per tick/s off the target speed (damping)

        Profile(String name, double ticksPerRev, double maxTicksPerSec,
                double kickDegrees, double maxDegrees,
                double softPower, double hardPower, double brakeDecel, double fireTimeout,
                double lowerSpeedDeg, double lowerMaxDown, double lowerMaxUp,
                int tolerance, double trackKp, double trackKd) {
            this.name = name;
            this.ticksPerRev = ticksPerRev;
            this.maxTicksPerSec = maxTicksPerSec;
            this.kickDegrees = kickDegrees;
            this.maxDegrees = maxDegrees;
            this.softPower = softPower;
            this.hardPower = hardPower;
            this.brakeDecel = brakeDecel;
            this.fireTimeout = fireTimeout;
            this.lowerSpeedDeg = lowerSpeedDeg;
            this.lowerMaxDown = lowerMaxDown;
            this.lowerMaxUp = lowerMaxUp;
            this.tolerance = tolerance;
            this.trackKp = trackKp;
            this.trackKd = trackKd;
        }
    }

    // goBILDA 5203 Yellow Jacket 1150 rpm (5.2:1), arm straight on the shaft.
    // 145.1 ticks/rev, 1150 x 145.1 / 60 = 2781 ticks/s. 90 deg = 36 ticks.
    // Shield at 180 deg. Every kick re-learns brakeDecel from what the brake
    // actually did (see learnBrake); this is only the first kick's value.
    // Once the brake was proven, HARD went to full power for the sharpest
    // take-off. 12000 then braked too early and cut the kick short, so the
    // start value went to 20000. For a longer push the stop angle went to 100
    // and the hard limit to 120 (still 60 deg short of the shield): with both
    // at 100, every tiny overshoot cut the brake value and slowed the next kick.
    public static final Profile ROBOT1_1150 = new Profile("robot1 1150rpm",
            145.1, 2781,
            100, 120,
            0.7, 1.0, 20000, 0.30,
            90, 0.30, 0.60,   // lower at 90 deg/s: about 1 s from the top
            3, 0.04, 0.002);  // 10 ticks behind -> 0.4 power

    // goBILDA 5203 6000 rpm (1:1). Only 28 ticks/rev (about 13 deg per tick)
    // and 2800 ticks/s: a whole kick is a handful of ticks and one or two
    // loops, so this robot will never stop as precisely as robot 1.
    public static final Profile ROBOT2_6000 = new Profile("robot2 6000rpm",
            28, 2800,
            90, 100,
            0.6, 1.0, 12000, 0.15,
            90, 0.20, 0.50,
            1, 0.15, 0.0005);

    public static final double COOLDOWN_TIME = 2.0;   // s after a kick before X can charge again
    static final double STOP_SPEED = 100;              // ticks/s; below this a braking arm counts as stopped
    static final double MAX_BRAKE_TIME = 0.15;         // s; reverse power never stays on longer than this
    static final double LATENCY_LOOPS = 1.0;           // how many loop times the reaction is late by
    static final double MISMATCH_DEGREES = 20;         // encoders further apart than this = motors fighting
    static final double LIMIT_GAP_DEGREES = 10;        // stop angle stays at least this far under maxDegrees
    static final double[] BRAKE_SCALES = {0.6, 1.0, 1.5, 2.0}; // Y cycles; x learned brake decel
    // Brake learning: next decel = LEARN_MARGIN x what the last brake achieved.
    // The measurement includes the reaction delay, so it already reads low;
    // 0.85 on top of that braked visibly early.
    static final double LEARN_MARGIN = 0.95;
    static final double LEARN_MIN_SPEED = 500;         // ticks/s; slower brakes are too noisy to learn from
    static final double MIN_DECEL = 5000, MAX_DECEL = 60000;
    static final double OVERSHOOT_CUT = 0.6;           // past maxDegrees: decel x this, brake much earlier

    enum State { READY, FIRING, COOLDOWN, FIRED, CHARGING, FAULT }
    enum Phase { ACCEL, BRAKE, DONE }

    private final DcMotorEx left, right;
    private final DcMotorEx[] motors;
    private final List<LynxModule> hubs;
    private final Profile p;
    private final ElapsedTime timer = new ElapsedTime();     // reset on every state change
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double loopTime = 0.010;                          // s, smoothed

    private State state = State.READY;
    private String fault = "";
    private boolean mismatch = false;  // latched: X can not clear it

    // Live trims (B, d-pad, Y). Lost when the OpMode restarts; copy good values into the profile.
    private double softPower, hardPower;
    private boolean hard = true; // start on the full-power kick
    private double kickDegrees;
    private int brakeScale = 1;
    private double learnedDecel; // ticks/s^2; starts at the profile value, updated after every kick
    private String learnNote = "not learned yet";

    // Shaft readings, refreshed every update().
    private int posL, posR, pos;
    private double vel;

    // The kick in progress / the last kick, for control and telemetry.
    private Phase phase = Phase.DONE;
    private int fireTicks;      // stop target, fixed when X is pressed
    private int peak;
    private int brakePos = -1;  // where braking started
    private double brakeVel;    // speed when braking started
    private double brakeAt;     // timer seconds when braking started

    // Holding after the kick and lowering.
    private int holdTarget;          // where the arm is held in COOLDOWN / FIRED
    private double lowerTarget;      // sliding target while CHARGING, ticks
    private double lowerTimeout;     // s
    private double dt;               // this loop's length, s

    public Kicker(HardwareMap hardwareMap, Profile profile) {
        p = profile;
        left = hardwareMap.get(DcMotorEx.class, "kick_left");
        right = hardwareMap.get(DcMotorEx.class, "kick_right");
        motors = new DcMotorEx[]{left, right};
        left.setDirection(LEFT_DIRECTION);
        right.setDirection(RIGHT_DIRECTION);

        // One bulk read per loop instead of one bus round trip per value.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        softPower = p.softPower;
        hardPower = p.hardPower;
        kickDegrees = Math.min(p.kickDegrees, p.maxDegrees - LIMIT_GAP_DEGREES);
        learnedDecel = p.brakeDecel;

        for (DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // never the hub's encoder modes
            m.setPower(0); // nothing moves during INIT
        }
    }

    /** Call once per loop. {@code xPressed} must be a rising edge, not the raw button. */
    public void update(boolean xPressed) {
        dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt < 0.2) loopTime = loopTime * 0.8 + dt * 0.2; // skip the first, long loop
        dt = Math.min(dt, 0.05);

        for (LynxModule hub : hubs) hub.clearBulkCache();
        posL = ENCODER_SIGN * left.getCurrentPosition();
        posR = ENCODER_SIGN * right.getCurrentPosition();
        pos = (posL + posR) / 2;
        vel = ENCODER_SIGN * (left.getVelocity() + right.getVelocity()) / 2;
        peak = Math.max(peak, pos);

        // At least 3 ticks so gear backlash on the 28-tick motor can not trip it.
        if (!mismatch && Math.abs(posL - posR) > Math.max(3, ticks(MISMATCH_DEGREES))) {
            mismatch = true;
            fail(String.format("encoders disagree (L %d, R %d): motors fighting - run kicker diag", posL, posR));
        }

        double t = timer.seconds();
        switch (state) {
            case READY:
                if (xPressed) fire();
                break;
            case FIRING:
                steer(t);
                if (phase == Phase.DONE) {
                    holdTarget = pos;
                    next(State.COOLDOWN);
                } else if (t >= p.fireTimeout) {
                    setPower(0);
                    phase = Phase.DONE;
                    if (pos < fireTicks / 4) {
                        // Barely moved: unplugged encoder or jammed arm.
                        fail(String.format("arm did not move (L %d, R %d of %d) - run kicker diag, X = pull back",
                                posL, posR, fireTicks));
                    } else {
                        holdTarget = pos;
                        next(State.COOLDOWN);
                    }
                }
                break;
            case COOLDOWN:
                track(holdTarget, 0);
                if (t >= COOLDOWN_TIME) {
                    learnBrake(); // the arm has long stopped, so peak is final
                    next(State.FIRED);
                }
                break;
            case FIRED:
                if (xPressed) charge();
                else track(holdTarget, 0);
                break;
            case CHARGING:
                double speed = p.lowerSpeedDeg / 360.0 * p.ticksPerRev; // ticks/s
                lowerTarget = Math.max(0, lowerTarget - speed * dt);
                if ((lowerTarget == 0 && reached(0) && Math.abs(vel) < STOP_SPEED) || t >= lowerTimeout) {
                    setPower(0);
                    next(State.READY);
                } else {
                    track(lowerTarget, lowerTarget > 0 ? -speed : 0);
                }
                break;
            case FAULT:
                // Motors are off. X pulls the arm back to the charged position so
                // the driver can try again, unless the motors were fighting.
                if (xPressed && !mismatch) charge();
                break;
        }
    }

    private void fire() {
        fireTicks = ticks(kickDegrees);
        phase = Phase.ACCEL;
        peak = pos;
        brakePos = -1;
        brakeVel = 0;
        setPower(hard ? hardPower : softPower);
        next(State.FIRING);
    }

    /** One loop of a kick. */
    private void steer(double t) {
        if (phase == Phase.ACCEL) {
            double v = Math.max(0, vel);
            double decel = learnedDecel * BRAKE_SCALES[brakeScale];
            double stopAt = pos + v * loopTime * LATENCY_LOOPS + v * v / (2 * decel);
            if (stopAt >= fireTicks || pos >= ticks(p.maxDegrees)) {
                setPower(-1.0);
                phase = Phase.BRAKE;
                brakePos = pos;
                brakeVel = v;
                brakeAt = t;
            }
        } else if (phase == Phase.BRAKE) {
            if (vel <= STOP_SPEED || t - brakeAt >= MAX_BRAKE_TIME) {
                setPower(0);
                phase = Phase.DONE;
            }
        }
    }

    /** Lower the arm slowly back to 0. The loop itself runs in update(). */
    private void charge() {
        phase = Phase.DONE;
        lowerTarget = Math.max(0, pos);
        double speed = p.lowerSpeedDeg / 360.0 * p.ticksPerRev;
        lowerTimeout = lowerTarget / speed + 2.0;
        next(State.CHARGING);
    }

    /** Follow a position (and speed) target. Down is limited harder than up. */
    private void track(double target, double targetVel) {
        double power = p.trackKp * (target - pos) + p.trackKd * (targetVel - vel);
        setPower(Math.max(-p.lowerMaxDown, Math.min(p.lowerMaxUp, power)));
    }

    public void stop() {
        setPower(0);
    }

    /** SOFT <-> HARD. Applies from the next kick. */
    public void toggleStrength() {
        hard = !hard;
    }

    /** Trims the strength that is currently picked. Applies from the next kick. */
    public void adjustFirePower(double delta) {
        if (hard) hardPower = clampPower(hardPower + delta);
        else      softPower = clampPower(softPower + delta);
    }

    private static double clampPower(double v) {
        return Math.round(Math.min(1.0, Math.max(0.1, v)) * 100) / 100.0;
    }

    /**
     * Trims the stop angle. It stays LIMIT_GAP_DEGREES under maxDegrees: a stop
     * angle right at the limit counts every small overshoot as a limit breach,
     * which cuts the learned brake and makes the next kick slower.
     */
    public void adjustKickAngle(double deltaDegrees) {
        kickDegrees = Math.min(p.maxDegrees - LIMIT_GAP_DEGREES, Math.max(15, kickDegrees + deltaDegrees));
    }

    /** brakeDecel x0.6 -> x1.0 -> x1.5 -> x2.0. Lower = brakes earlier. */
    public void cycleBrakeScale() {
        brakeScale = (brakeScale + 1) % BRAKE_SCALES.length;
    }

    public void addTelemetry(Telemetry telemetry) {
        String status = state.toString();
        if (state == State.COOLDOWN) {
            status += String.format(" (%.1f s)", Math.max(0, COOLDOWN_TIME - timer.seconds()));
        } else if (state == State.READY) {
            status += " - X to kick";
        } else if (state == State.FIRED) {
            status += " - holding, X to lower";
        } else if (state == State.CHARGING) {
            status += String.format(" - lowering, target %.0f deg", lowerTarget * 360.0 / p.ticksPerRev);
        } else if (state == State.FAULT) {
            status += " - " + fault;
        }
        telemetry.addData("kicker", "%s", status);
        telemetry.addData("kick power", "%s %.2f  (B soft/hard, dpad up/down)  soft %.2f / hard %.2f",
                hard ? "HARD" : "SOFT", hard ? hardPower : softPower, softPower, hardPower);
        telemetry.addData("kick angle", "%.0f deg = %d ticks  (dpad left/right, up to %.0f, limit %.0f)",
                kickDegrees, ticks(kickDegrees), p.maxDegrees - LIMIT_GAP_DEGREES, p.maxDegrees);
        telemetry.addData("brake decel", "%.0f t/s^2  (learned %.0f, Y: x%.1f)  loop %.1f ms",
                learnedDecel * BRAKE_SCALES[brakeScale], learnedDecel, BRAKE_SCALES[brakeScale], loopTime * 1000);
        telemetry.addData("brake learning", "%s", learnNote);
        telemetry.addData("last kick", "peak %.0f deg | brake from %.0f deg at %.0f deg/s | measured decel %.0f t/s^2",
                degrees(peak), degrees(Math.max(0, brakePos)), brakeVel * 360.0 / p.ticksPerRev, measuredDecel());
        telemetry.addData("kick pos", "L %d  R %d  (charged 0)  [%s]", posL, posR, p.name);
    }

    /**
     * After a kick: aim the next brake at what this one really achieved.
     * Too-early brakes (arm stopped short) raise the value, too-late ones lower it.
     * Lost when the OpMode restarts; copy a settled value into the profile.
     */
    private void learnBrake() {
        if (peak > ticks(p.maxDegrees)) {
            learnedDecel = Math.max(MIN_DECEL, learnedDecel * OVERSHOOT_CUT);
            learnNote = String.format("OVERSHOOT to %.0f deg -> cut to %.0f", degrees(peak), learnedDecel);
            return;
        }
        double measured = measuredDecel();
        if (brakePos < 0 || brakeVel < LEARN_MIN_SPEED || measured <= 0) {
            learnNote = "last kick too slow to learn from, kept " + Math.round(learnedDecel);
            return;
        }
        double before = learnedDecel;
        learnedDecel = Math.max(MIN_DECEL, Math.min(MAX_DECEL, measured * LEARN_MARGIN));
        learnNote = String.format("measured %.0f -> %.0f (was %.0f)", measured, learnedDecel, before);
    }

    /** Decel actually achieved on the last kick, ticks/s^2; 0 if unknown. */
    private double measuredDecel() {
        int dist = peak - brakePos;
        if (brakePos < 0 || dist <= 0 || brakeVel <= 0) return 0;
        return brakeVel * brakeVel / (2.0 * dist);
    }

    private int ticks(double deg) {
        return Math.max(1, (int) Math.round(deg / 360.0 * p.ticksPerRev));
    }

    private double degrees(int ticks) {
        return ticks * 360.0 / p.ticksPerRev;
    }

    private boolean reached(int target) {
        return Math.abs(posL - target) <= p.tolerance
            && Math.abs(posR - target) <= p.tolerance;
    }

    private void setPower(double power) {
        left.setPower(power);
        right.setPower(power);
    }

    private void fail(String why) {
        stop();
        phase = Phase.DONE;
        fault = why;
        next(State.FAULT);
    }

    private void next(State s) {
        state = s;
        timer.reset();
    }
}
