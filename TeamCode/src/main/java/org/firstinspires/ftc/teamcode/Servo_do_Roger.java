package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="servo_do_roger", group="Linear OpMode")
public class servo_do_roger extends LinearOpMode {

    private Servo rampa;

    @Override
    public void runOpMode() {
        rampa = hardwareMap.get(Servo.class, "rampa");

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                rampa.setPosition(0.0);
            } else if (gamepad1.b) {
                rampa.setPosition(1.0);
            }

            telemetry.addData("Posição Servo", rampa.getPosition());
            telemetry.update();
        }
    }
}
