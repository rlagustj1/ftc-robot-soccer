package com.example.teamcode.command;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Command-based replacement for mecanum_drive.java.
 *
 * gamepad1 left stick   : translation, any angle, any magnitude
 * gamepad1 right stick x: rotation
 * gamepad1 right bumper : slow mode
 * gamepad1 dpad up      : hold to creep straight forward (demo of a command
 *                         interrupting the default one)
 */
@TeleOp(name = "cmd mecanum_drive", group = "cmd")
public class MecanumTeleOp extends CommandOpMode {

    static final double CREEP_POWER = 0.25;

    private DriveSubsystem drive;
    private GamepadEx driver;

    @Override
    public void initialize() {
        // CommandScheduler is a singleton. Drop whatever a previous OpMode left
        // behind BEFORE the subsystem registers itself with the new instance.
        reset();

        drive = new DriveSubsystem(hardwareMap);
        driver = new GamepadEx(gamepad1);

        // GamepadEx.getLeftY() already returns -left_stick_y, so pushing the
        // stick forward gives a positive value.
        DefaultDriveCommand driveCommand = new DefaultDriveCommand(
                drive,
                driver::getLeftX,
                driver::getLeftY,
                driver::getRightX,
                () -> gamepad1.right_bumper,
                telemetry);

        drive.setDefaultCommand(driveCommand);

        // Both commands require the drivetrain, so holding dpad-up interrupts
        // the joystick command and releasing it hands control straight back.
        driver.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whileHeld(new DriveDirectionCommand(drive, 0, 1, 0, CREEP_POWER));

        telemetry.addLine("Left stick = move, right stick = turn, RB = slow.");
        telemetry.update();
    }
}
