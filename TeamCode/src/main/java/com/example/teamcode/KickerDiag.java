package com.example.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * Kicker check. Run this before trusting Kicker.
 *
 * Both motors drive ONE shared shaft, so both encoders must always agree.
 *
 * 1. Hand test (no buttons): push the arm by hand from the charged position
 *    forward by 90 deg. BOTH encoders must go UP by about 36 ticks (1150 rpm,
 *    145.1 ticks/rev). One going DOWN means that motor's direction in Kicker
 *    is wrong and the two motors fight each other. One not moving means a bad
 *    encoder cable. Neither can be fixed by tuning.
 * 2. Jog test (hold a button): drives ONE motor at JOG_POWER (the other just
 *    turns along). "Forward" on either motor must move the arm forward and
 *    both encoders up. If one motor's "forward" moves the arm backwards, that
 *    is the motor to flip.
 *
 * gamepad1 dpad up / down : left motor forward / back
 * gamepad1 Y / A          : right motor forward / back
 * gamepad1 X              : zero both encoders here
 *
 * Uses the same names, directions and ENCODER_SIGN as Kicker, so with the
 * right settings: arm pushed UP by hand -> both values go UP, and jogging a
 * motor "forward" (dpad up / Y) moves the arm UP and both values go UP.
 */
@TeleOp(name = "kicker diag", group = "test")
public class KickerDiag extends LinearOpMode {

    static final double JOG_POWER = 0.15;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx left = hardwareMap.get(DcMotorEx.class, "kick_left");
        DcMotorEx right = hardwareMap.get(DcMotorEx.class, "kick_right");
        left.setDirection(Kicker.LEFT_DIRECTION);
        right.setDirection(Kicker.RIGHT_DIRECTION);

        for (DcMotorEx m : new DcMotorEx[]{left, right}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // easy to turn by hand
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setPower(0);
        }

        telemetry.addLine("Arm CHARGED (pulled back), then START.");
        telemetry.addLine("Push the arm UP (the kick direction) 90 deg by hand: BOTH values should go to about +36.");
        telemetry.update();
        waitForStart();

        int peakL = 0, peakR = 0;
        while (opModeIsActive()) {
            if (gamepad1.xWasPressed()) {
                for (DcMotorEx m : new DcMotorEx[]{left, right}) {
                    m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
                peakL = peakR = 0;
            }

            double lp = gamepad1.dpad_up ? JOG_POWER : gamepad1.dpad_down ? -JOG_POWER : 0;
            double rp = gamepad1.y ? JOG_POWER : gamepad1.a ? -JOG_POWER : 0;
            left.setPower(lp);
            right.setPower(rp);

            int l = Kicker.ENCODER_SIGN * left.getCurrentPosition();
            int r = Kicker.ENCODER_SIGN * right.getCurrentPosition();
            peakL = Math.max(peakL, l);
            peakR = Math.max(peakR, r);

            telemetry.addData("LEFT  (port 0)", "%5d ticks  %6.1f deg  max %5.1f deg  %6.0f t/s  %.2f A  pwr %.2f",
                    l, deg(l), deg(peakL), Kicker.ENCODER_SIGN * left.getVelocity(), left.getCurrent(CurrentUnit.AMPS), lp);
            telemetry.addData("RIGHT (port 1)", "%5d ticks  %6.1f deg  max %5.1f deg  %6.0f t/s  %.2f A  pwr %.2f",
                    r, deg(r), deg(peakR), Kicker.ENCODER_SIGN * right.getVelocity(), right.getCurrent(CurrentUnit.AMPS), rp);
            telemetry.addData("L - R", "%d ticks  (should stay near 0: one shaft)", l - r);
            telemetry.addLine("90 deg forward = +36 ticks (1150) / +7 ticks (6000)");
            telemetry.addLine("dpad up/down = left jog, Y/A = right jog, X = zero");
            telemetry.update();
        }
    }

    /** Degrees for the 1150 rpm motor (145.1 ticks/rev). */
    private static double deg(int ticks) {
        return ticks * 360.0 / 145.1;
    }
}
