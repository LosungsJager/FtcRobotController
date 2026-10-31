package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "genau", group = "Linear Opmode")
public class genau extends LinearOpMode {

    private DcMotor esq_f = null;
    private DcMotor esq_t = null;
    private DcMotor dir_f = null;
    private DcMotor dir_t = null;
    private DcMotor lancador = null;
    private DcMotor elastico = null;
    private Servo servo = null;


    final double POSICAO_RECOLHIDO = 0.0;
    final double POSICAO_EMPURRAR = 0.5;


    long tempoLiberacaoLancador = 0;

    @Override
    public void runOpMode() {

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        elastico = hardwareMap.get(DcMotor.class, "elastico");
        servo = hardwareMap.get(Servo.class, "servo");


        esq_f.setDirection(DcMotor.Direction.FORWARD);
        esq_t.setDirection(DcMotor.Direction.REVERSE);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        dir_t.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);
        elastico.setDirection(DcMotor.Direction.FORWARD);


        esq_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esq_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Status", "V3 Carregado - Proteção Ativa");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {


            if (gamepad1.b) {
                esq_f.setPower(0);
                esq_t.setPower(0);
                dir_f.setPower(0);
                dir_t.setPower(0);
                lancador.setPower(0);
                elastico.setPower(0);
                servo.setPosition(POSICAO_RECOLHIDO);
                telemetry.addData("ALERTA", "PARADA DE EMERGÊNCIA");
                telemetry.update();
                continue;
            }


            double velocidade = gamepad1.left_bumper ? 1.0 : 0.5;
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            esq_f.setPower(((y + x + rx) / denominator) * velocidade);
            esq_t.setPower(((y - x + rx) / denominator) * velocidade);
            dir_f.setPower(((y - x - rx) / denominator) * velocidade);
            dir_t.setPower(((y + x - rx) / denominator) * velocidade);




            double comandoDesejado = 0.0;
            if (gamepad2.right_trigger > 0.1) {
                comandoDesejado = 1.0;  // Disparar
            } else if (gamepad2.left_bumper) {
                comandoDesejado = -0.5; // Reverso
            }


            double potenciaAtual = lancador.getPower();


            boolean mudancaBrusca = (potenciaAtual > 0.1 && comandoDesejado < -0.1) ||
                    (potenciaAtual < -0.1 && comandoDesejado > 0.1);

            if (mudancaBrusca) {

                lancador.setPower(0);

                tempoLiberacaoLancador = System.currentTimeMillis() + 500;
            }
            else {

                if (System.currentTimeMillis() > tempoLiberacaoLancador) {

                    lancador.setPower(comandoDesejado);
                } else {

                    lancador.setPower(0);
                }
            }

            //coletor
            if (gamepad2.a) elastico.setPower(0.8);
            else if (gamepad2.b) elastico.setPower(-0.8);
            else elastico.setPower(0);

            // servo
            if (gamepad2.x) servo.setPosition(POSICAO_EMPURRAR);
            else servo.setPosition(POSICAO_RECOLHIDO);


            telemetry.addData("Lançador Power", lancador.getPower());
            if (System.currentTimeMillis() < tempoLiberacaoLancador) {
                telemetry.addData("PROTEÇÃO", "AGUARDANDO FREIO...");
            }
            telemetry.update();
        }
    }
}
