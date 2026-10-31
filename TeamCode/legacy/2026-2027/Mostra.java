package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Mostra", group="Demo")
public class Mostra extends LinearOpMode {

    private DcMotor esq_f, esq_t, dir_f, dir_t, elevador;
    private Servo pa;

    @Override
    public void runOpMode() {
        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        elevador = hardwareMap.get(DcMotor.class, "elevador");
        pa = hardwareMap.get(Servo.class, "paddle");

        esq_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esq_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double esq_fPower = (y + x + rx) / denominator;
            double esq_tPower = (y - x + rx) / denominator;
            double dir_fPower = (y - x - rx) / denominator;
            double dir_tPower = (y + x - rx) / denominator;

            double velocidade = 1.0;
            if (gamepad1.left_bumper) velocidade = 0.5;
            if (gamepad1.right_bumper) velocidade = 1.0;

            esq_f.setPower(esq_fPower * velocidade);
            esq_t.setPower(esq_tPower * velocidade);
            dir_f.setPower(dir_fPower * velocidade);
            dir_t.setPower(dir_tPower * velocidade);

            if (gamepad1.dpad_up) {
                elevador.setPower(0.6);
            } else if (gamepad1.dpad_down) {
                elevador.setPower(-0.6);;
            } else {
                elevador.setPower(0);
            }

            double stick = gamepad2.left_stick_x;
            stick = Math.max(-1.0, Math.min(1.0, stick));
            double posServo = (stick + 1.0) / 2.0;
            pa.setPosition(posServo);

            telemetry.addLine("Controle Único Completo");
            telemetry.addData("Velocidade Geral", "%.2f", velocidade);
            telemetry.addData("Elevador Pos", elevador.getCurrentPosition());
            telemetry.addData("Servo Pos", pa.getPosition());
            telemetry.addData("Motores",
                    "EsqF: %.2f | EsqT: %.2f | DirF: %.2f | DirT: %.2f",
                    esq_fPower, esq_tPower, dir_fPower, dir_tPower);
            telemetry.update();
        }
    }
 }
