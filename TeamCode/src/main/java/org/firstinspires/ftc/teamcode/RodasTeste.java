package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class RodasTeste extends LinearOpMode {
    public DcMotor esq_f, dir_f,esq_t, dir_t = null;

    IMU.Parameters myIMUparameters;

    BHI260IMU imu;

    public void rotacao() {
        //Rotação motores tração.
        esq_f.setDirection(DcMotor.Direction.FORWARD);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        esq_t.setDirection(DcMotor.Direction.FORWARD);
        dir_t.setDirection(DcMotor.Direction.REVERSE);

    }

    @Override
    public void runOpMode() throws InterruptedException {
        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        imu = hardwareMap.get(BHI260IMU.class, "imu");

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        imu.initialize(parameters);

        if (gamepad1.options) {
            imu.resetYaw();
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        waitForStart();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; //o y é inverso, por isso o -
            double rx = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x * 1.1; //corrigir imperfeições do movimento lateral

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            //O denominador é a maior potência do motor (valor absoluto) ou 1
            // Isso garante que todas as potências mantenham a mesma proporção,
            // mas somente se pelo menos uma estiver fora do intervalo [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            /*double esq_fPower = (rotY + rotX + rx) / denominator;
            double esq_tPower = (rotY - rotX + rx) / denominator;
            double dir_fPower = (rotY - rotX - rx) / denominator;
            double dir_tPower = (rotY + rotX - rx) / denominator;*/

            double esq_fPower = (y + x + rx) / denominator;
            double esq_tPower = (y -x - rx) / denominator;
            double dir_fPower = (-(y - x +rx)) / denominator;
            double dir_tPower = (y + x - rx) / denominator;

            esq_f.setPower(esq_fPower);
            esq_t.setPower(esq_tPower);
            dir_f.setPower(dir_fPower);
            dir_t.setPower(dir_tPower) ;

        }
    }
}
