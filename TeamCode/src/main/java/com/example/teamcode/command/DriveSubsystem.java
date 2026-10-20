package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum drivetrain.
 *
 * The wheel sign patterns are exactly the ones used by the plain-SDK OpModes
 * in com.example.teamcode:
 *
 *   forward -> straight  (lf + , rf + , lb + , rb +)
 *   strafe  -> right     (lf + , rf - , lb - , rb +)
 *   turn    -> r_r       (lf + , rf - , lb + , rb -)
 *
 * Configure names must match exactly: "motor1" .. "motor4".
 */
public class DriveSubsystem extends SubsystemBase {

    private final Motor l_f, r_f, l_b, r_b;

    public DriveSubsystem(HardwareMap hMap) {
        r_f = new Motor(hMap, "motor1");
        r_b = new Motor(hMap, "motor2");
        l_b = new Motor(hMap, "motor3");
        l_f = new Motor(hMap, "motor4");

        // Same effect as DcMotor.Direction.REVERSE on the left side.
        l_f.setInverted(true);
        l_b.setInverted(true);

        for (Motor m : new Motor[]{l_f, r_f, l_b, r_b}) {
            m.setRunMode(Motor.RunMode.RawPower);
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }
    }

    /**
     * Drives one fixed motion mix, scaled so the strongest wheel gets exactly
     * {@code power}. This reproduces the single-direction OpModes: passing
     * (0, 1, 0) gives straight, (1, 1, 0) gives d_fr, and so on.
     */
    public void driveScaled(double strafe, double forward, double turn, double power) {
        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double peak = peak(lf, rf, lb, rb);
        double k = (peak > 1e-9) ? power / peak : 0.0;

        set(lf * k, rf * k, lb * k, rb * k);
    }

    /**
     * TeleOp drive. The travel direction (ux, uy) is taken from the stick and
     * never altered; only mag decides how fast. Rotation is added afterwards.
     *
     * @param ux   x component of the travel direction (unit vector, 0 when idle)
     * @param uy   y component of the travel direction
     * @param mag  how fast to travel in that direction, 0..1
     * @param turn rotation, -1..1 (positive = clockwise)
     */
    public void driveTeleop(double ux, double uy, double mag, double turn) {
        double lf = uy + ux;
        double rf = uy - ux;
        double lb = uy - ux;
        double rb = uy + ux;

        // Full stick in ANY direction -> strongest wheel reaches mag.
        double peak = peak(lf, rf, lb, rb);
        if (peak > 1e-9) {
            double k = mag / peak;
            lf *= k;
            rf *= k;
            lb *= k;
            rb *= k;
        }

        lf -= turn;
        rf += turn;
        lb -= turn;
        rb += turn;

        // Translation plus rotation can exceed 1.0; keep the ratios.
        double over = Math.max(1.0, peak(lf, rf, lb, rb));
        set(lf / over, rf / over, lb / over, rb / over);
    }

    public void stop() {
        set(0, 0, 0, 0);
    }

    /** lf, rf, lb, rb - for telemetry. */
    public double[] getPowers() {
        return new double[]{l_f.get(), r_f.get(), l_b.get(), r_b.get()};
    }

    private void set(double lf, double rf, double lb, double rb) {
        l_f.set(lf);
        r_f.set(rf);
        l_b.set(lb);
        r_b.set(rb);
    }

    private static double peak(double a, double b, double c, double d) {
        return Math.max(Math.abs(a), Math.max(Math.abs(b),
               Math.max(Math.abs(c), Math.abs(d))));
    }
}
