package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Perry", group = "1_Lösungsjäger")
public class Perry extends LinearOpMode {

    private static final double REDUCAO = 5.0;

    private static final double DIAMETRO_RODA_MM = 104.0;

    private static final double TICKS_MOTOR = 28.0;

    private static final double TICKS_POR_VOLTA = TICKS_MOTOR * REDUCAO;

    private static final double RPM_LIVRE_MOTOR = 6000.0;

    private double velocidadeMaxima = 0.80;
    private static final double VEL_MINIMA = 0.25;
    private static final double VEL_MAXIMA = 1.00;
    private static final double PASSO_AJUSTE = 0.025;
    private static final double FATOR_PRECISAO = 0.3;

    private static final double ZONA_MORTA = 0.06;

    private static final double POTENCIA_INTAKE = 0.95; // Inverta o sinal se o intake girar ao contrário.
    private static final double POTENCIA_SAIDA = -1.7;

    // Ajuste fino da tração. Comece em 1.00; aumente a roda fraca em passos de 0.02.
    private static final double AJUSTE_ESQ_F = 0.20;
    private static final double AJUSTE_ESQ_T = 0.30;
    private static final double AJUSTE_DIR_F = 0.50;
    private static final double AJUSTE_DIR_T = 0.70;

    private static final double POS_TRAVAR_RAMPA = 0.2;

    private static final double PEGAR_RAMPA = 0.78;

    // Corrige a tendência de virar à esquerda durante avanço/ré.
    // Valor positivo aplica uma leve correção para a direita.
    private static final double TRIM_GIRO_AVANCO = 0.2;

    private DcMotorEx esq_f, esq_t, dir_f, dir_t, giro1, giro2, intake;   // esquerda/direita, frente/trás
    private Servo rampa1, rampa2;

    @Override
    public void runOpMode() {

        iniciarHardware();

        telemetry.addLine("Perry pronto.");
        telemetry.addLine("Mantenha o robô PARADO até apertar START.");
        telemetry.addData("Redução configurada", "%.2f:1", REDUCAO);
        telemetry.addData("RPM estimado na roda", "%.0f", RPM_LIVRE_MOTOR / REDUCAO);
        telemetry.addData("Velocidade livre estimada", "%.2f m/s", velocidadeLivreMs());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            atualizarAjusteVelocidade();
            dirigir();
            controlarIntake();
            mostrarTelemetria();
        }
    }

    private void iniciarHardware() {

        esq_f = hardwareMap.get(DcMotorEx.class, "esq_f");
        esq_t = hardwareMap.get(DcMotorEx.class, "esq_t");
        dir_f = hardwareMap.get(DcMotorEx.class, "dir_f");
        dir_t = hardwareMap.get(DcMotorEx.class, "dir_t");
        giro1 = hardwareMap.get(DcMotorEx.class, "giro1");
        giro2 = hardwareMap.get(DcMotorEx.class, "giro2");
        intake = hardwareMap.get(DcMotorEx.class, "int");
        rampa1 = hardwareMap.get(Servo.class, "rampa1");
        rampa2 = hardwareMap.get(Servo.class, "rampa2");

        // lado esquerdo gira invertido
        esq_f.setDirection(DcMotorSimple.Direction.REVERSE);
        esq_t.setDirection(DcMotorSimple.Direction.REVERSE);
        dir_f.setDirection(DcMotorSimple.Direction.FORWARD);
        dir_t.setDirection(DcMotorSimple.Direction.FORWARD);
        giro1.setDirection(DcMotorSimple.Direction.REVERSE);
        giro2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        rampa1.setPosition(POS_TRAVAR_RAMPA);
        rampa2.setPosition(POS_TRAVAR_RAMPA);

        for (DcMotorEx m : new DcMotorEx[]{esq_f, esq_t, dir_f, dir_t, giro1, giro2, intake}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void atualizarAjusteVelocidade() {
        if (gamepad1.right_trigger > 0.05) {
            velocidadeMaxima += PASSO_AJUSTE * gamepad1.right_trigger;
        }
        if (gamepad1.left_trigger > 0.05) {
            velocidadeMaxima -= PASSO_AJUSTE * gamepad1.left_trigger;
        }
        velocidadeMaxima = Math.max(VEL_MINIMA, Math.min(VEL_MAXIMA, velocidadeMaxima));
    }

    private void dirigir() {

        double avanco =  gamepad1.left_stick_y;
        double lado   =  gamepad1.left_stick_x *1.1 ;
        double giro   =  gamepad1.right_stick_x;


        avanco = zonaMorta(avanco);
        lado   = zonaMorta(lado);
        giro   = zonaMorta(giro);

        // Quando o piloto não comanda giro, mantém o trajeto reto durante avanço ou ré.
        if (Math.abs(giro) < ZONA_MORTA && Math.abs(avanco) >= ZONA_MORTA) {
            giro = TRIM_GIRO_AVANCO * avanco;
        }

        double pEsqF = avanco + lado + giro;
        double pEsqT = avanco - lado + giro;
        double pDirF = avanco - lado - giro;
        double pDirT = avanco + lado - giro;

        // Compensa diferenças mecânicas entre rodas antes da normalização.
        pEsqF *= AJUSTE_ESQ_F;
        pEsqT *= AJUSTE_ESQ_T;
        pDirF *= AJUSTE_DIR_F;
        pDirT *= AJUSTE_DIR_T;

        double maior = Math.max(1.0, Math.max(
                Math.max(Math.abs(pEsqF), Math.abs(pEsqT)),
                Math.max(Math.abs(pDirF), Math.abs(pDirT))));

        double fator = velocidadeMaxima;
        if (gamepad1.left_stick_button) {
            fator *= FATOR_PRECISAO;
        }

        if (Math.abs(gamepad1.left_stick_x) < 0.05 &&
            Math.abs(gamepad1.left_stick_y) < 0.05 &&
            Math.abs(gamepad1.right_stick_x) < 0.05) {

            esq_f.setPower(0);
            esq_t.setPower(0);
            dir_f.setPower(0);
            dir_t.setPower(0);
        } else {

            esq_f.setPower(pEsqF / maior * fator);
            esq_t.setPower(pEsqT / maior * fator);
            dir_f.setPower(pDirF / maior * fator);
            dir_t.setPower(pDirT / maior * fator);

        }
    }


    private void controlarIntake() {
        if (gamepad2.right_bumper) {
            intake.setPower(POTENCIA_INTAKE);
        } else {
            intake.setPower(0.0);
        }
        if (gamepad2.left_bumper) {
            giro1.setPower(POTENCIA_SAIDA / 25);
            giro2.setPower(-POTENCIA_SAIDA / 25);
        } else if (gamepad2.left_trigger > 0.1) {
            giro1.setPower(POTENCIA_SAIDA / 50);
            giro2.setPower(-POTENCIA_SAIDA / 50);
        } else{
            giro1.setPower(0.0);
            giro2.setPower(0.0);
        }
        if (gamepad2.a) {
            rampa1.setPosition(PEGAR_RAMPA);
            rampa2.setPosition(PEGAR_RAMPA);
        } else if (gamepad2.x) {
            rampa1.setPosition(POS_TRAVAR_RAMPA);
            rampa2.setPosition(POS_TRAVAR_RAMPA);
        }
    }

    private double zonaMorta(double valor) {
        return Math.abs(valor) < ZONA_MORTA ? 0.0 : valor;
    }

    private double velocidadeLivreMs() {
        double rpmRoda = RPM_LIVRE_MOTOR / REDUCAO;
        double circunferenciaM = Math.PI * DIAMETRO_RODA_MM / 1000.0;
        return rpmRoda / 60.0 * circunferenciaM;
    }


    private void mostrarTelemetria() {
        telemetry.addData("Velocidade", "%.0f%%%s",
                velocidadeMaxima * 100,
                gamepad1.left_stick_button ? "  [PRECISÃO]" : "");
        telemetry.addData("Ticks/volta na roda", "%.0f", TICKS_POR_VOLTA);
        telemetry.addData("Ajuste rodas", "EF %.2f | ET %.2f | DF %.2f | DT %.2f",
                AJUSTE_ESQ_F, AJUSTE_ESQ_T, AJUSTE_DIR_F, AJUSTE_DIR_T);
        telemetry.addData("Trim de giro", "%.2f", TRIM_GIRO_AVANCO);
        telemetry.addLine();
        telemetry.addData("Intake (RB)", gamepad2.right_bumper ? "GIRANDO" : "parado");
        telemetry.addData("Saída (LB)", gamepad2.left_bumper ? "GIRANDO" : "parada");
        telemetry.addData("Precisão (pressionar L3)", gamepad1.left_stick_button ? "ATIVA" : "desativada");
        telemetry.update();
    }
}
