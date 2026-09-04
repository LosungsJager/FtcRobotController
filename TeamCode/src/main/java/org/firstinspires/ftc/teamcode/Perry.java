package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


@TeleOp(name = "Perry", group = "1_Lösungsjäger")
public class Perry extends LinearOpMode {

    private static final double REDUCAO = 5.23;

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

    private static final double LIMITE_POTENCIA_INTAKE = -0.75;   // 0.0 a 1.0

    private static final double GANHO_CORRECAO = 0.2;
    private static final double GIRO_MAX_CORRECAO = 0.2;

    private DcMotorEx esq_f, esq_t, dir_f, dir_t, col;   // esquerda/direita, frente/trás
    private GoBildaPinpointDriver pinpoint;

    private boolean fieldCentric = false;
    private boolean yAnterior = false;
    private double headingAlvo = 0.0;
    private boolean travandoHeading = false;

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
            pinpoint.update();

            atualizarAjusteVelocidade();
            atualizarModoDirecao();
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
        col = hardwareMap.get(DcMotorEx.class, "col");

        // lado esquerdo gira invertido
        esq_f.setDirection(DcMotorSimple.Direction.REVERSE);
        esq_t.setDirection(DcMotorSimple.Direction.REVERSE);
        dir_f.setDirection(DcMotorSimple.Direction.FORWARD);
        dir_t.setDirection(DcMotorSimple.Direction.FORWARD);
        col.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx m : new DcMotorEx[]{esq_f, esq_t, dir_f, dir_t}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        col.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        col.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.setOffsets(0.0, 0.0, DistanceUnit.MM);

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        // Calibra o IMU interno — o robô PRECISA estar imóvel aqui.
        pinpoint.resetPosAndIMU();
        sleep(300);


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

    private void atualizarModoDirecao() {
        boolean yAgora = gamepad1.y;
        if (yAgora && !yAnterior) {
            fieldCentric = !fieldCentric;
        }
        yAnterior = yAgora;
        if (gamepad1.start) {
            pinpoint.resetPosAndIMU();
        }
    }

    private void dirigir() {

        double avanco = -gamepad1.left_stick_y;
        double lado   =  gamepad1.left_stick_x;
        double giro   =  gamepad1.right_stick_x;


        avanco = zonaMorta(avanco);
        lado   = zonaMorta(lado);
        giro   = zonaMorta(giro);

        double headingAtual = pinpoint.getHeading(AngleUnit.RADIANS);

        if (Math.abs(giro) > 0.001) {
            travandoHeading = false;
        } else {
            if (!travandoHeading) {
                headingAlvo = headingAtual;
                travandoHeading = true;
            }
            double erro = anguloNormalizado(headingAlvo - headingAtual);
            double correcao = erro * GANHO_CORRECAO / 0.5; // escala compatível com o loop
            correcao = Math.max(-GIRO_MAX_CORRECAO, Math.min(GIRO_MAX_CORRECAO, correcao));
            giro = correcao;
        }

        if (fieldCentric) {
            double cos = Math.cos(-headingAtual);
            double sen = Math.sin(-headingAtual);

            double novoLado   = lado * cos - avanco * sen;
            double novoAvanco = lado * sen + avanco * cos;

            lado = novoLado;
            avanco = novoAvanco;
        }

        lado *= 1.1;

        double pEsqF = avanco + lado + giro;
        double pEsqT = avanco - lado + giro;
        double pDirF = avanco - lado - giro;
        double pDirT = avanco + lado - giro;

        double maior = Math.max(1.0, Math.max(
                Math.max(Math.abs(pEsqF), Math.abs(pEsqT)),
                Math.max(Math.abs(pDirF), Math.abs(pDirT))));

        double fator = velocidadeMaxima;
        if (gamepad1.left_bumper) {
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
        if (gamepad1.right_bumper) {
            col.setPower(LIMITE_POTENCIA_INTAKE);
        } else {
            col.setPower(0.0);
        }
    }

    private double zonaMorta(double valor) {
        return Math.abs(valor) < ZONA_MORTA ? 0.0 : valor;
    }

    private double anguloNormalizado(double radianos) {
        while (radianos > Math.PI) radianos -= 2 * Math.PI;
        while (radianos < -Math.PI) radianos += 2 * Math.PI;
        return radianos;
    }

    private double velocidadeLivreMs() {
        double rpmRoda = RPM_LIVRE_MOTOR / REDUCAO;
        double circunferenciaM = Math.PI * DIAMETRO_RODA_MM / 1000.0;
        return rpmRoda / 60.0 * circunferenciaM;
    }


    private void mostrarTelemetria() {
        Pose2D pos = pinpoint.getPosition();

        telemetry.addData("Modo", fieldCentric ? "FIELD-CENTRIC" : "robot-centric");
        telemetry.addData("Velocidade", "%.0f%%%s",
                velocidadeMaxima * 100,
                gamepad1.left_bumper ? "  [PRECISÃO]" : "");
        telemetry.addLine();
        telemetry.addData("Posição X", "%.1f mm", pos.getX(DistanceUnit.MM));
        telemetry.addData("Posição Y", "%.1f mm", pos.getY(DistanceUnit.MM));
        telemetry.addData("Ângulo", "%.1f°", pos.getHeading(AngleUnit.DEGREES));
        telemetry.addLine();
        telemetry.addData("Ticks/volta na roda", "%.0f", TICKS_POR_VOLTA);
        telemetry.addData("Status Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addLine();
        telemetry.addData("Heading-lock", travandoHeading ? "ATIVO (corrigindo)" : "livre (girando)");
        telemetry.addLine();
        telemetry.addData("Coletor", gamepad1.right_bumper ? "GIRANDO" : "parado");
        telemetry.addData("Limite de potência do coletor", "%.0f%%", LIMITE_POTENCIA_INTAKE * 100);
        telemetry.update();
    }
}