package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class teste_lancarBolinha extends LinearOpMode {

    private DcMotor lancador, elastico_1, elastico_2 = null;

    @Override
    public void runOpMode() throws InterruptedException {
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        elastico_1 = hardwareMap.get(DcMotor.class, "elastico_1");
        elastico_2 = hardwareMap.get(DcMotor.class, "elastico_2");

        lancador.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lancador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        elastico_1.setDirection(DcMotor.Direction.FORWARD);
        elastico_2.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_up) {
                lancador.setPower(0.5);
            } else if (gamepad1.dpad_down) {
                lancador.setPower(-0.5);
            } else {
                lancador.setPower(0);
            }

            double stick = gamepad1.left_stick_x;
            stick = Math.max(-1.0, Math.min(1.0, stick));

            elastico_1.setPower(gamepad2.left_stick_y);
            elastico_2.setPower(-gamepad2.right_stick_y);

            telemetry.addData("lancador", lancador.getCurrentPosition());
            telemetry.update();
        }
    }
}
