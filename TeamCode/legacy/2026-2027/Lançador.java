package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Lançador", group = "Competition")
public class Lançador extends LinearOpMode {

    private DcMotor launcher;
    private DcMotor escovas;
    private Servo rampa;
    private Servo cano;

    private static final double POS_ENTRADA = 0.2;
    private static final double POS_TRAVAR_RAMPA = 0.5;
    private static final double POS_LANCAR = 0.8;

    private static final double Aberto = 0.0;
    private static final double Fechado = 0.3;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        escovas = hardwareMap.get(DcMotor.class, "escovas");
        rampa = hardwareMap.get(Servo.class, "rampa");
        cano = hardwareMap.get(Servo.class, "cano");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        escovas.setDirection(DcMotor.Direction.FORWARD);

        rampa.setPosition(POS_TRAVAR_RAMPA);
        cano.setPosition(Aberto);

        telemetry.addLine("Pronto. Aguarde início...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_trigger > 0.5) {
                launcher.setPower(1.0);
            } else if (gamepad1.b) {
                launcher.setPower(-1.0);
            } else {
                launcher.setPower(0.0);
            }

            double feedPower = gamepad1.left_stick_y;
            escovas.setPower(feedPower);

            if (gamepad1.a) {
                rampa.setPosition(POS_ENTRADA);
            } else if (gamepad1.x) {
                rampa.setPosition(POS_TRAVAR_RAMPA);
            } else if (gamepad1.y) {
                rampa.setPosition(POS_LANCAR);
            }

            if (gamepad2.a) {
                cano.setPosition(Aberto);
            } else if (gamepad2.b) {
                cano.setPosition(Fechado);
            }

            telemetry.addData("Launcher", launcher.getPower());
            telemetry.addData("Feed", feedPower);
            telemetry.addData("Rampa Pos", rampa.getPosition());
            telemetry.addData("Tampa Pos", cano.getPosition());
            telemetry.update();
        }
    }
}
