package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class safira extends LinearOpMode {
    public DcMotor esq_f, dir_f, esq_t, dir_t, lancador, elastico_1, elastico_2 = null;

    private IMU imu;


    private static final double LIMIT = 0.55;
    private static final double TURBO = 1.00;
    private static final double SHOOTER_LIMIT = 1.2;

    public void rotacao() {
        //Rotação motores tração.
        esq_f.setDirection(DcMotor.Direction.FORWARD);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        esq_t.setDirection(DcMotor.Direction.FORWARD);
        dir_t.setDirection(DcMotor.Direction.REVERSE);
        elastico_1.setDirection(DcMotor.Direction.FORWARD);
        elastico_2.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        elastico_1 = hardwareMap.get(DcMotor.class, "elastico_1");
        elastico_2 = hardwareMap.get(DcMotor.class, "elastico_2");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        if (gamepad1.options) {
            imu.resetYaw();
        }

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.options) imu.resetYaw();

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double speed = gamepad1.right_bumper ? TURBO : LIMIT;

            double y = -gamepad1.left_stick_y; //o y é inverso, por isso o -
            double x = gamepad1.left_stick_x * 1.1; //corrigir imperfeições do movimento lateral
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0; //medida de segurança caso o joystick acabe mexendo um pouco sozinho
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX = rotX * 1.1;

            //O denominador é a maior potência do motor (valor absoluto) ou 1
            // Isso garante que todas as potências mantenham a mesma proporção,
            // mas somente se pelo menos uma estiver fora do intervalo [-1, 1]

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double esq_fPower = (rotY - rotX - rx) / denominator;
            double esq_tPower = (rotY + rotX/1.4 - rx) / denominator;
            double dir_fPower = (rotY + rotX + rx) / denominator;
            double dir_tPower = (-rotY + rotX - rx) / denominator;

            esq_f.setPower(esq_fPower);
            esq_t.setPower(esq_tPower);
            dir_f.setPower(dir_fPower);
            dir_t.setPower(dir_tPower);

            if (gamepad2.right_trigger > 0.5) lancador.setPower(- SHOOTER_LIMIT);
            else if (gamepad2.left_trigger > 0.5) lancador.setPower(-0.4);
            else lancador.setPower(0);

            elastico_1.setPower(gamepad2.left_stick_y);
            elastico_2.setPower(-gamepad2.right_stick_y);

            telemetry.addData("Drive", speed == TURBO ? "Turbo" : "Normal");
            telemetry.addData("Lançador", lancador.getPower());
            telemetry.addData("Elástico inf", elastico_1.getPower());
            telemetry.addData("Elástico sup", elastico_2.getPower());
            telemetry.update();

        }
    }
}
