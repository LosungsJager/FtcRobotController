package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

@Autonomous(name="Rubi Estados")
public class rubiEstado extends LinearOpMode {

    DcMotor esq_f, esq_t, dir_f, dir_t;
    DcMotor lancador, elastico;
    IMU imu;

    enum Estado {
        ANDAR,
        LIGAR_LANCADOR,
        ATIVAR_ELASTICO,
        FINALIZAR
    }

    Estado estado = Estado.ANDAR;
    long tempoEstado;

    @Override
    public void runOpMode() {

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");

        lancador = hardwareMap.get(DcMotor.class, "lancador");
        elastico = hardwareMap.get(DcMotor.class, "elastico");

        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )));

        esq_f.setDirection(DcMotor.Direction.FORWARD);
        esq_t.setDirection(DcMotor.Direction.FORWARD);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        dir_t.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        tempoEstado = System.currentTimeMillis();

        while (opModeIsActive()) {

            long agora = System.currentTimeMillis();
            long tempo = agora - tempoEstado;

            switch (estado) {

                case ANDAR:
                    setDrive(-0.4);
                    if (tempo >= 500) {
                        pararDrive();
                        estado = Estado.LIGAR_LANCADOR;
                        tempoEstado = agora;
                    }
                    break;

                case LIGAR_LANCADOR:
                    lancador.setPower(-1);
                    if (tempo >= 4000) {
                        estado = Estado.ATIVAR_ELASTICO;
                        tempoEstado = agora;
                    }
                    break;

                case ATIVAR_ELASTICO:
                    elastico.setPower(0.8);
                    if (tempo >= 2000) {
                        estado = Estado.FINALIZAR;
                    }
                    break;

                case FINALIZAR:
                    pararTudo();
                    break;
            }

            telemetry.addData("Estado", estado);
            telemetry.update();
        }
    }

    void setDrive(double p) {
        esq_f.setPower(-p);
        esq_t.setPower(-p);
        dir_f.setPower(p);
        dir_t.setPower(p);
    }

    void pararDrive() {
        esq_f.setPower(0);
        esq_t.setPower(0);
        dir_f.setPower(0);
        dir_t.setPower(0);
    }

    void pararTudo() {
        pararDrive();
        lancador.setPower(0);
        elastico.setPower(0);
    }
}
