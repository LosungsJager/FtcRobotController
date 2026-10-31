package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DECODE", group = "Competition")
public class DECODE extends LinearOpMode {

    private DcMotor esq_f, dir_f, esq_t, dir_t;
    private BHI260IMU imu;

    private DcMotor launcher;
    private DcMotor escovas;
    private Servo rampa;
    private Servo cano;

    private static final double POS_ENTRADA = 0.2;
    private static final double POS_TRAVAR_RAMPA = 0.5;
    private static final double POS_LANCAR = 0.8;
    private static final double CANO_ABERTO = 0.0;
    private static final double CANO_FECHADO = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        escovas = hardwareMap.get(DcMotor.class, "escovas");
        rampa = hardwareMap.get(Servo.class, "rampa");
        cano = hardwareMap.get(Servo.class, "cano");

        esq_f.setDirection(DcMotor.Direction.REVERSE);
        dir_f.setDirection(DcMotor.Direction.FORWARD);
        esq_t.setDirection(DcMotor.Direction.REVERSE);
        dir_t.setDirection(DcMotor.Direction.FORWARD);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        escovas.setDirection(DcMotor.Direction.FORWARD);

        rampa.setPosition(POS_TRAVAR_RAMPA);
        cano.setPosition(CANO_ABERTO);

        telemetry.addLine("Pronto. Aguarde início...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double esq_fPower = (rotY + rotX - rx) / denominator *4; //tem que testar isso
            double esq_tPower = (-rotY + rotX + rx) / denominator;
            double dir_fPower = (-rotY - rotX - rx) / denominator;
            double dir_tPower = (-rotY + rotX - rx) / denominator *4;

            esq_f.setPower(esq_fPower);
            esq_t.setPower(esq_tPower);
            dir_f.setPower(dir_fPower);
            dir_t.setPower(dir_tPower);


            if (gamepad2.right_trigger > 0.5) {
                launcher.setPower(1.0);
            } else if (gamepad2.b) {
                launcher.setPower(-1.0);
            } else {
                launcher.setPower(0.0);
            }

            double feedPower = -gamepad2.left_stick_y;
            escovas.setPower(feedPower);

            if (gamepad2.a) {
                rampa.setPosition(POS_ENTRADA);
            } else if (gamepad2.x) {
                rampa.setPosition(POS_TRAVAR_RAMPA);
            } else if (gamepad2.y) {
                rampa.setPosition(POS_LANCAR);
            }

            if (gamepad2.dpad_up) {
                cano.setPosition(CANO_ABERTO);
            } else if (gamepad2.dpad_down) {
                cano.setPosition(CANO_FECHADO);
            }

            telemetry.addData("Chassi", "EsqF(%.2f), DirF(%.2f), EsqT(%.2f), DirT(%.2f)", esq_fPower, dir_fPower, esq_tPower, dir_tPower);
            telemetry.addData("Ângulo (Yaw)", "%.2f Graus", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("---", "---");
            telemetry.addData("Lançador", launcher.getPower());
            telemetry.addData("Escovas", feedPower);
            telemetry.addData("Rampa Posição", rampa.getPosition());
            telemetry.addData("Cano Posição", cano.getPosition());
            telemetry.update();
        }
    }
}
