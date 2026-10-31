package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo_Test_FTC", group = "Test")
public class testeServo extends LinearOpMode {

    Servo servo;

    // Posições seguras (ajuste se quiser)
    double POSICAO_1 = 0.5;
    double POSICAO_2 = 0.8;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");

        // posição inicial
        servo.setPosition(POSICAO_1);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo.setPosition(POSICAO_1);
            }

            if (gamepad1.b) {
                servo.setPosition(POSICAO_2);
            }

            telemetry.addData("Posição atual", servo.getPosition());
            telemetry.update();
        }
    }
}
