package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class teste_pegarBolinha extends LinearOpMode {
    public DcMotor rolamento = null;

    IMU.Parameters myIMUparameters;

    BHI260IMU imu;

    public void rotacao() {
        //Rotação motores tração.
        rolamento.setDirection(DcMotor.Direction.FORWARD);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        rolamento = hardwareMap.get(DcMotor.class, "rolamento");
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

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad2.dpad_up){
                rolamento.setPower(-100);
            } else if(gamepad1.dpad_down){
                rolamento.setPower(100);
            } else {
                rolamento.setPower(0);
            }
        }
    }
}
