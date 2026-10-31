package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Elevador", group="Linear OpMode")
public class Elevador2 extends LinearOpMode {

    private DcMotor movelevador = null;
    private Servo pa = null;

    @Override
    public void runOpMode() throws InterruptedException {
        movelevador = hardwareMap.get(DcMotor.class, "elevador");
        pa = hardwareMap.get(Servo.class, "paddle");

        movelevador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        movelevador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        movelevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Pronto. Aguarde início...");
        telemetry.update();

        waitForStart();

        
        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                movelevador.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                movelevador.setPower(-0.5);
            } else {
                movelevador.setPower(0);
            }

            double stick = gamepad1.left_stick_x;
            stick = Math.max(-1.0, Math.min(1.0, stick));
            double posServo = (stick + 1.0) / 2.0;
            pa.setPosition(posServo);

            telemetry.addData("Elevador", movelevador.getCurrentPosition());
            telemetry.addData("Servo (posição)", pa.getPosition());
            telemetry.update();
        }


        telemetry.addData("Posição Elevador", movelevador.getCurrentPosition());
            telemetry.addData("Servo (posição)", pa.getPosition());
            telemetry.update();
        }
    }
