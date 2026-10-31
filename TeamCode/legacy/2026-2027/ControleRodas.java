package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Rodas", group="Linear OpMode")
public class ControleRodas extends LinearOpMode {
    public DcMotor esq_f, dir_f, esq_t, dir_t = null;
    IMU.Parameters myIMUparameters;
    BHI260IMU imu;
    double fatorEsqF = 0.8;
    double fatorEsqT = 1.0;
    double fatorDirF = 1.0;
    double fatorDirT = 1.0;
    final double AJUSTE_PASSO = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x * 1.1;
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            double esq_fPower = (y + x + rx) / denominator * fatorEsqF;
            double esq_tPower = (y - x - rx) / denominator * fatorEsqT;
            double dir_fPower = (-(y - x + rx)) / denominator * fatorDirF;
            double dir_tPower = (y + x - rx) / denominator * fatorDirT;

            esq_f.setPower(esq_fPower);
            esq_t.setPower(esq_tPower);
            dir_f.setPower(dir_fPower);
            dir_t.setPower(dir_tPower);

            if (gamepad1.dpad_up) fatorEsqF += AJUSTE_PASSO;
            if (gamepad1.dpad_down) fatorEsqF -= AJUSTE_PASSO;
            if (gamepad1.dpad_right) fatorEsqT += AJUSTE_PASSO;
            if (gamepad1.dpad_left) fatorEsqT -= AJUSTE_PASSO;
            if (gamepad2.dpad_up) fatorDirF += AJUSTE_PASSO;
            if (gamepad2.dpad_down) fatorDirF -= AJUSTE_PASSO;
            if (gamepad2.dpad_right) fatorDirT += AJUSTE_PASSO;
            if (gamepad2.dpad_left) fatorDirT -= AJUSTE_PASSO;

            telemetry.addData("Fator Esq F", "%.2f", fatorEsqF);
            telemetry.addData("Fator Esq T", "%.2f", fatorEsqT);
            telemetry.addData("Fator Dir F", "%.2f", fatorDirF);
            telemetry.addData("Fator Dir T", "%.2f", fatorDirT);
            telemetry.update();
        }
    }
}
