package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "DECODE3", group = "Competition")
public class DECODE3 extends LinearOpMode {

    private DcMotor esq_f, esq_t, dir_f, dir_t, elastico_1, elastico_2, lancador;
    private IMU imu;

    private static final double LIMIT = 0.55;
    private static final double TURBO = 1.00;
    private static final double SHOOTER_LIMIT = 1.2;

    @Override
    public void runOpMode() {

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");

        elastico_1 = hardwareMap.get(DcMotor.class, "elastico_1");
        elastico_2 = hardwareMap.get(DcMotor.class, "elastico_2");
        lancador  = hardwareMap.get(DcMotor.class, "lancador");

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        esq_f.setDirection(DcMotor.Direction.REVERSE);
        esq_t.setDirection(DcMotor.Direction.REVERSE);
        dir_f.setDirection(DcMotor.Direction.FORWARD);
        dir_t.setDirection(DcMotor.Direction.FORWARD);

        elastico_1.setDirection(DcMotor.Direction.FORWARD);
        elastico_2.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);

        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.options) imu.resetYaw();

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double speed = gamepad1.right_bumper ? TURBO : LIMIT;

            double y  = -gamepad1.left_stick_y * speed;
            double x  =  gamepad1.left_stick_x * 1.1 * speed;
            double rx =  gamepad1.right_stick_x * speed;

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double d = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

            esq_f.setPower((rotY + rotX - rx) / d);
            esq_t.setPower((rotY - rotX - rx) / d);
            dir_f.setPower((-rotY - rotX - rx) / d);
            dir_t.setPower((-rotY + rotX - rx) / d);

            if (gamepad2.right_trigger > 0.5) lancador.setPower(- SHOOTER_LIMIT);
            else if (gamepad2.left_trigger > 0.5) lancador.setPower(-0.4);
            else lancador.setPower(0);

            elastico_1.setPower(gamepad2.left_stick_y);
            elastico_2.setPower(-gamepad2.right_stick_y);

            telemetry.addData("Drive", speed == TURBO ? "Turbo" : "Normal");
            telemetry.addData("Lançador", lancador.getPower());
            telemetry.addData("Elástico 1", elastico_1.getPower());
            telemetry.addData("Elástico 2", elastico_2.getPower());
            telemetry.update();
        }
    }
}
