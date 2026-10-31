package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@TeleOp(name = "Topazio", group = "Competition")
public class Topazio extends LinearOpMode {
    private static final double LIMIT = 0.55;
    private static final double TURBO = 1.0;
    private static final double SHOOTER_LIMIT = 1.2;
    private static final double DEADZONE = 0.05;
    private static final double TAG_ALIGN_KP = 0.025;
    private static final double TAG_ALIGN_KD = 0.002;
    private static final double TAG_ALIGN_MAX = 0.5;

    private DcMotor esq_f;
    private DcMotor dir_f;
    private DcMotor esq_t;
    private DcMotor dir_t;
    private DcMotor lancador;
    private DcMotor elastico;
    private IMU imu;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private final ElapsedTime alignTimer = new ElapsedTime();
    private double lastError = 0.0;

    private void configurarMotores() {
        esq_f.setDirection(DcMotor.Direction.FORWARD);
        esq_t.setDirection(DcMotor.Direction.FORWARD);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        dir_t.setDirection(DcMotor.Direction.REVERSE);
        elastico.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        lancador = hardwareMap.get(DcMotor.class, "lancador");
        elastico = hardwareMap.get(DcMotor.class, "elastico");
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        configurarMotores();

        waitForStart();
        alignTimer.reset();

        while (opModeIsActive()) {
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double speed = gamepad1.right_bumper ? TURBO : LIMIT;

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < DEADZONE) y = 0;
            if (Math.abs(x) < DEADZONE) x = 0;
            if (Math.abs(rx) < DEADZONE) rx = 0;

            if (gamepad1.left_trigger > 0.5) {
                AprilTagDetection detection = getTargetTag();
                if (detection != null && detection.ftcPose != null) {
                    double erro = -detection.ftcPose.bearing;
                    double dt = Math.max(alignTimer.seconds(), 0.02);
                    double derivada = (erro - lastError) / dt;
                    double ajuste = (erro * TAG_ALIGN_KP) + (derivada * TAG_ALIGN_KD);
                    rx = Math.max(-TAG_ALIGN_MAX, Math.min(TAG_ALIGN_MAX, ajuste));
                    lastError = erro;
                    alignTimer.reset();
                }
            }

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double esq_fPower = (rotY - rotX - rx) / denominator;
            double esq_tPower = (rotY + rotX / 1.4 - rx) / denominator;
            double dir_fPower = (rotY + rotX + rx) / denominator;
            double dir_tPower = (-rotY + rotX - rx) / denominator;

            esq_f.setPower(esq_fPower * speed);
            esq_t.setPower(esq_tPower * speed);
            dir_f.setPower(dir_fPower * speed);
            dir_t.setPower(dir_tPower * speed);

            if (gamepad2.right_trigger > 0.5) {
                lancador.setPower(-SHOOTER_LIMIT);
            } else if (gamepad2.left_trigger > 0.5) {
                lancador.setPower(-0.4);
            } else {
                lancador.setPower(0);
            }

            elastico.setPower(gamepad2.left_stick_y);

            telemetry.addData("Drive", speed == TURBO ? "Turbo" : "Normal");
            telemetry.addData("Lançador", lancador.getPower());
            telemetry.addData("Elástico", elastico.getPower());
            telemetry.addData("AutoAlign", gamepad1.left_trigger > 0.5 ? "Tag" : "Manual");
            telemetry.update();
        }

        visionPortal.close();
    }

    private AprilTagDetection getTargetTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) {
            return null;
        }
        return detections.get(0);
    }
}
