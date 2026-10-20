package com.example.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Test OpMode: pressing INIT on the Driver Station immediately spins both motors.
 * Configure names must match exactly: "motor1" (Control Hub port 0), "motor2" (port 1).
 */
@TeleOp(name = "back")
public class back extends LinearOpMode {

    static final double POWER = 0.5; // -1.0 to 1.0

    public void back(DcMotor l_f,DcMotor r_f,DcMotor l_b, DcMotor r_b){
        l_f.setPower(-POWER);
        r_f.setPower(-POWER);
        l_b.setPower(-POWER);
        r_b.setPower(-POWER);
    }


    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor r_f = hardwareMap.get(DcMotor.class, "motor1");
        DcMotor r_b = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor l_b = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor l_f = hardwareMap.get(DcMotor.class, "motor4");


        l_f.setDirection(DcMotor.Direction.REVERSE);
        l_b.setDirection(DcMotor.Direction.REVERSE);



        // Runs as soon as INIT is pressed, before waitForStart().
       back(l_f,r_f,l_b,r_b);


        waitForStart();

        while (opModeIsActive()) {
            idle();
        }

        l_f.setPower(0);
        l_b.setPower(0);
        r_b.setPower(0);
        r_f.setPower(0);

    }

}
