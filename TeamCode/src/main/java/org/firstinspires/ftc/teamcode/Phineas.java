package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Phineas", group = "1_Lösungsjäger")
public class Phineas extends LinearOpMode {

    // Tração
    private DcMotorEx esqF, esqT, dirF, dirT, giro1,giro2, intake;

    private Servo rampa1, rampa2;
    private static final double ZONA_MORTA = 0.06;

    private static final double VELOCIDADE_INICIAL = 0.80;
    private static final double VELOCIDADE_MINIMA = 0.25;
    private static final double VELOCIDADE_MAXIMA = 1.00;
    private static final double AJUSTE_VELOCIDADE_POR_SEGUNDO = 0.50;

    private static final double FATOR_PRECISAO = 0.30;

    private static final double POTENCIA_INTAKE = 0.70;
    private static final double POTENCIA_SAIDA = -0.65;
    private static final double POTENCIA_TESTE_GIRO = 0.30;

    private static final double POS_TRAVAR_RAMPA = 0.20;
    private static final double POS_PEGAR_RAMPA = 0.78;

    private double velocidadeMaxima = VELOCIDADE_INICIAL;

    @Override
    public void runOpMode() {
        mapearHardware();
        configurarHardware();

        telemetry.addLine("Perry Completo pronto.");
        telemetry.addLine("Gamepad 1: direção, intake e saída.");
        telemetry.addLine("Gamepad 2: rampa e teste individual dos giros.");
        telemetry.addLine("Mantenha o robô parado até apertar START.");
        telemetry.update();

        waitForStart();

        long ultimoTempoNs = System.nanoTime();

        while (opModeIsActive()) {
            long agoraNs = System.nanoTime();
            double deltaSegundos = (agoraNs - ultimoTempoNs) / 1_000_000_000.0;
            ultimoTempoNs = agoraNs;

            ajustarVelocidade(deltaSegundos);
            controlarTracao();
            controlarIntakeESaida();
            controlarRampa();
            atualizarTelemetria();

            idle();
        }

        pararMotores();
    }

    private void mapearHardware() {
        esqF = hardwareMap.get(DcMotorEx.class, "esq_f");
        esqT = hardwareMap.get(DcMotorEx.class, "esq_t");
        dirF = hardwareMap.get(DcMotorEx.class, "dir_f");
        dirT = hardwareMap.get(DcMotorEx.class, "dir_t");

        giro1 = hardwareMap.get(DcMotorEx.class, "giro1");
        giro2 = hardwareMap.get(DcMotorEx.class, "giro2");
        intake = hardwareMap.get(DcMotorEx.class, "int");

        rampa1 = hardwareMap.get(Servo.class, "rampa1");
        rampa2 = hardwareMap.get(Servo.class, "rampa2");
    }

    private void configurarHardware() {
        esqF.setDirection(DcMotorSimple.Direction.REVERSE);
        esqT.setDirection(DcMotorSimple.Direction.REVERSE);
        dirF.setDirection(DcMotorSimple.Direction.FORWARD);
        dirT.setDirection(DcMotorSimple.Direction.FORWARD);

        giro1.setDirection(DcMotorSimple.Direction.REVERSE);
        giro2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotorEx[] motores = {
                esqF, esqT, dirF, dirT,
                giro1, giro2, intake
        };

        for (DcMotorEx motor : motores) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
        }

        rampa1.setPosition(POS_TRAVAR_RAMPA);
        rampa2.setPosition(POS_TRAVAR_RAMPA);
    }

    private void ajustarVelocidade(double deltaSegundos) {
        if (gamepad1.right_trigger > 0.05) {
            velocidadeMaxima += AJUSTE_VELOCIDADE_POR_SEGUNDO
                    * gamepad1.right_trigger * deltaSegundos;
        }

        if (gamepad1.left_trigger > 0.05) {
            velocidadeMaxima -= AJUSTE_VELOCIDADE_POR_SEGUNDO
                    * gamepad1.left_trigger * deltaSegundos;
        }

        velocidadeMaxima = Math.max(
                VELOCIDADE_MINIMA,
                Math.min(VELOCIDADE_MAXIMA, velocidadeMaxima)
        );
    }

    private void controlarTracao() {
        double avanco = -gamepad1.left_stick_y;
        double lado = gamepad1.left_stick_x;
        double giro = gamepad1.right_stick_x;

        avanco = aplicarZonaMorta(avanco);
        lado = aplicarZonaMorta(lado);
        giro = aplicarZonaMorta(giro);

        double pEsqF = avanco + lado + giro;
        double pEsqT = avanco - lado + giro;
        double pDirF = avanco - lado - giro;
        double pDirT = avanco + lado - giro;

        double maior = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(pEsqF), Math.abs(pEsqT)),
                        Math.max(Math.abs(pDirF), Math.abs(pDirT))
                )
        );

        double fator = velocidadeMaxima;
        if (gamepad1.left_stick_button) {
            fator *= FATOR_PRECISAO;
        }

        esqF.setPower((pEsqF / maior) * fator);
        esqT.setPower((pEsqT / maior) * fator);
        dirF.setPower((pDirF / maior) * fator);
        dirT.setPower((pDirT / maior) * fator);
    }

    private void controlarIntakeESaida() {

        intake.setPower(
                gamepad1.right_bumper ? POTENCIA_INTAKE : 0.0
        );

        if (gamepad2.left_bumper) {
            giro1.setPower(POTENCIA_SAIDA);
            giro2.setPower(-POTENCIA_SAIDA);
        } else {
            giro1.setPower(0.0);
            giro2.setPower(0.0);
        }
    }

    private void controlarRampa() {
        if (gamepad2.a) {
            rampa1.setPosition(POS_PEGAR_RAMPA);
            rampa2.setPosition(POS_PEGAR_RAMPA);
        }

        if (gamepad2.x) {
            rampa1.setPosition(POS_TRAVAR_RAMPA);
            rampa2.setPosition(POS_TRAVAR_RAMPA);
        }
    }

    private double aplicarZonaMorta(double valor) {
        return Math.abs(valor) < ZONA_MORTA ? 0.0 : valor;
    }

    private void atualizarTelemetria() {
        telemetry.addData("Velocidade máxima", "%.0f%%", velocidadeMaxima * 100);
        telemetry.addData("Precisão", gamepad1.left_stick_button ? "ATIVA" : "desativada");
        telemetry.addData("Intake (G1 RB)", gamepad1.right_bumper ? "ligado" : "parado");
        telemetry.addData("Saída conjunta (G1 LB)", gamepad1.left_bumper ? "ligada" : "parada");
        telemetry.addData("Teste giro1 (G2 B)", gamepad2.b ? "ligado" : "parado");
        telemetry.addData("Teste giro2 (G2 Y)", gamepad2.y ? "ligado" : "parado");
        telemetry.addData("Rampa (G2 A/X)", "A = pegar | X = travar");
        telemetry.update();
    }

    private void pararMotores() {
        esqF.setPower(0.0);
        esqT.setPower(0.0);
        dirF.setPower(0.0);
        dirT.setPower(0.0);

        giro1.setPower(0.0);
        giro2.setPower(0.0);
        intake.setPower(0.0);
    }
}
