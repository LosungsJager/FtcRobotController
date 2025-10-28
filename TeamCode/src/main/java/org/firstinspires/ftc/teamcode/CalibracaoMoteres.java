package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "CalibracaoMotores", group = "Test")
public class CalibracaoMotores extends LinearOpMode {

    DcMotor esq_f, esq_t, dir_f, dir_t;

    double potEsqF = 0.5;
    double potEsqT = 0.5;
    double potDirF = 0.5;
    double potDirT = 0.5;

    int motorSelecionado = 0;
    final double AJUSTE_PASSO = 0.01;

    @Override
    public void runOpMode() throws InterruptedException {

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");

        esq_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esq_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Pronto para calibrar");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) motorSelecionado = 0;
            if (gamepad1.b) motorSelecionado = 1;
            if (gamepad1.x) motorSelecionado = 2;
            if (gamepad1.y) motorSelecionado = 3;

            if (gamepad1.dpad_up) {
                if (motorSelecionado == 0) potEsqF += AJUSTE_PASSO;
                if (motorSelecionado == 1) potEsqT += AJUSTE_PASSO;
                if (motorSelecionado == 2) potDirF += AJUSTE_PASSO;
                if (motorSelecionado == 3) potDirT += AJUSTE_PASSO;
            }
            if (gamepad1.dpad_down) {
                if (motorSelecionado == 0) potEsqF -= AJUSTE_PASSO;
                if (motorSelecionado == 1) potEsqT -= AJUSTE_PASSO;
                if (motorSelecionado == 2) potDirF -= AJUSTE_PASSO;
                if (motorSelecionado == 3) potDirT -= AJUSTE_PASSO;
            }

            potEsqF = Math.max(-1.0, Math.min(1.0, potEsqF));
            potEsqT = Math.max(-1.0, Math.min(1.0, potEsqT));
            potDirF = Math.max(-1.0, Math.min(1.0, potDirF));
            potDirT = Math.max(-1.0, Math.min(1.0, potDirT));

            esq_f.setPower(potEsqF);
            esq_t.setPower(potEsqT);
            dir_f.setPower(potDirF);
            dir_t.setPower(potDirT);

            telemetry.addLine("=== MODO CALIBRACAO ===");
            telemetry.addData("Motor Selecionado", nomeMotorSelecionado());
            telemetry.addData("Pot Frente Esq", "%.2f", potEsqF);
            telemetry.addData("Pot Tras Esq", "%.2f", potEsqT);
            telemetry.addData("Pot Frente Dir", "%.2f", potDirF);
            telemetry.addData("Pot Tras Dir", "%.2f", potDirT);
            telemetry.update();
        }
    }

    private String nomeMotorSelecionado() {
        switch (motorSelecionado) {
            case 0: return "Frente Esquerda";
            case 1: return "Traseira Esquerda";
            case 2: return "Frente Direita";
            case 3: return "Traseira Direita";
        }
        return "Desconhecido";
    }
}
