package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TwoMotorTeleOp")
public class TwoMotorTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Robot Controller 앱의 Configuration에서
        // 포트 1 모터 이름 → "motor1", 포트 2 모터 이름 → "motor2" 로 설정해야 합니다.
        Motor motor1 = new Motor(hardwareMap, "motor1");
        Motor motor2 = new Motor(hardwareMap, "motor2");

        motor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        motor1.setRunMode(Motor.RunMode.RawPower);
        motor2.setRunMode(Motor.RunMode.RawPower);

        // 두 모터가 좌우 대칭으로 장착된 경우 한쪽을 반전 (필요 없으면 이 줄 삭제)
        motor2.setInverted(true);

        GamepadEx driver = new GamepadEx(gamepad1);

        waitForStart();

        while (opModeIsActive()) {
            // 탱크 드라이브: 왼쪽 스틱 상하 → motor1, 오른쪽 스틱 상하 → motor2
            motor1.set(driver.getLeftY());
            motor2.set(driver.getRightY());

            telemetry.addData("motor1 power", motor1.get());
            telemetry.addData("motor2 power", motor2.get());
            telemetry.update();
        }

        motor1.stopMotor();
        motor2.stopMotor();
    }
}
