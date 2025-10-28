package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class imu extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("esq_f");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("esq_t");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("dir_f");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("dir_t");

        // trocar o sinal do direito
        // se andar pra tras mudar o esquerdo
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Recuperar a IMU do mapa de hardware
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // ajustar o parametro de orientação para combinar com o robo
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // sem isso, o orientador do REV Hub é assumido (não consegui traduzir) to be logo up / USB forward
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //inverti os sinais, a frente foi trocada
            double y = gamepad1.left_stick_y; // y stick é inverso
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // é para segurança
            // pode ser tirado, é opicional
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rota o contador da direção do movimento para a rotação do bot
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 0.6;  // neutralize a imperfeição strafing

            // Denominator é o maior potencia do motor (valor absoluto) ou 1
            // Isso garante que todas as potencias mantenham a mesma proporção,
            // mas somente se pelo menos um estiver fora do intervalo [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (-rotY + rotX - rx) / denominator * 6.2;
            double backLeftPower = (-rotY - rotX + rx) / denominator * 2;
            double frontRightPower = (-rotY - rotX - rx) / denominator * 3.2;
            double backRightPower = (rotY - rotX - rx) / denominator * 2;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
    }
}
