/*package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "LadoAzulEb", group = "Competition")
public class teste extends LinearOpMode {
    private static final double SHOOTER_LIMIT = 1.2;

    private DcMotor esq_f;
    private DcMotor dir_f;
    private DcMotor esq_t;
    private DcMotor dir_t;
    private DcMotor lancador;
    private DcMotor elastico;
    private IMU imu;

    private VisionPortal visionPortal;
    private ColorBlobProcessor colorProcessor;
    private AprilTagProcessor aprilTag;

    //20 azul 24 vermelho
    private final int ALVO_APRILTAG = 20;

    private void configurarMotores() {
        esq_f.setDirection(DcMotor.Direction.FORWARD);
        esq_t.setDirection(DcMotor.Direction.FORWARD);
        dir_f.setDirection(DcMotor.Direction.REVERSE);
        dir_t.setDirection(DcMotor.Direction.REVERSE);
        elastico.setDirection(DcMotor.Direction.FORWARD);
        lancador.setDirection(DcMotor.Direction.FORWARD);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        esq_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esq_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dir_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setDrivePower(double fl, double fr, double bl, double br, long tempoMs) {
        esq_f.setPower(fl);
        dir_f.setPower(fr);
        esq_t.setPower(bl);
        dir_t.setPower(br);
        sleep(tempoMs);
        pararMotores();
    }

    private void setMecanumPower(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;


        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max; fr /= max; bl /= max; br /= max;
        }

        esq_f.setPower(fl);
        dir_f.setPower(fr);
        esq_t.setPower(bl);
        dir_t.setPower(br);
    }

    private void pararMotores() {
        esq_f.setPower(0);
        dir_f.setPower(0);
        esq_t.setPower(0);
        dir_t.setPower(0);
    }

    private void moverFrente(double power, long tempoMs) { setDrivePower(power, power, power, power, tempoMs); }
    private void moverTras(double power, long tempoMs) { setDrivePower(-power, -power, -power, -power, tempoMs); }
    private void moverLado(double power, long tempoMs) { setDrivePower(power, -power, -power, power, tempoMs); }
    private void girar(double power, long tempoMs) { setDrivePower(power, -power, power, -power, tempoMs); }

    private void disparar() {
        lancador.setPower(-SHOOTER_LIMIT);LadoAzulEb
        sleep(gamepad1.nextRumbleApproxFinishTime);
        elastico.sd
        sleep(1200);
        lancador.setPower(0);
    }

    private void autoAlignToGoal(int tagId) {
        final double DESIRED_DISTANCE = 40; //polegadas/cm
        final double KP_X = 0.02;
        final double KP_Y = 0.02;
        final double KP_YAW = 0.01;

        long timeout = System.currentTimeMillis() + 4000;

        while (opModeIsActive() && System.currentTimeMillis() < timeout) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean tagEncontrada = false;

            for (AprilTagDetection tag : detections) {
                if (tag.metadata != null && tag.id == tagId) {
                    tagEncontrada = true;

                    double errorX = tag.ftcPose.x;
                    double errorY = tag.ftcPose.y - DESIRED_DISTANCE;
                    double errorYaw = tag.ftcPose.yaw;

                    if (Math.abs(errorX) < 1.5 && Math.abs(errorY) < 2.5 && Math.abs(errorYaw) < 2.0) {
                        pararMotores();
                        return;
                    }

                    double strafe = errorX * KP_X;
                    double forward = errorY * KP_Y;
                    double turn = -errorYaw * KP_YAW;

                    setMecanumPower(forward, strafe, turn);
                }
            }

            if (!tagEncontrada) {
                pararMotores();
            }
            sleep(20);
        }
        pararMotores();
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

        configurarMotores();

        colorProcessor = new ColorBlobProcessor();
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(colorProcessor)
                .addProcessor(aprilTag)
                .build();

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("Status", "Pronto");
            telemetry.addData("Alvo Cor", colorProcessor.getPosicao());
            telemetry.addData("Dist Cor", colorProcessor.getDistanciaCm());
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));

            boolean tagVisivel = false;
            for(AprilTagDetection d : aprilTag.getDetections()) {
                if(d.id == ALVO_APRILTAG) tagVisivel = true;
            }
            telemetry.addData("Tag " + ALVO_APRILTAG + " Visível?", tagVisivel ? "Sim" : "Não");

            telemetry.update();
            sleep(20);
        }

        waitForStart();

        if (isStopRequested()) return;

        String posicao = colorProcessor.getPosicao();

        visionPortal.setProcessorEnabled(colorProcessor, false);

        if ("ESQUERDA".equals(posicao)) rotaEsquerda();
        else if ("CENTRO".equals(posicao)) rotaCentro();
        else if ("DIREITA".equals(posicao)) rotaDireita();
        else rotaPadrao();

        visionPortal.close();
    }

    private void rotaEsquerda() {
        moverFrente(0.5, 900);
        moverLado(0.5, 600);
        autoAlignToGoal(ALVO_APRILTAG);
        disparar();
        estacionar();
    }

    private void rotaCentro() {
        moverFrente(0.6, 1100);
        autoAlignToGoal(ALVO_APRILTAG);
        disparar();
        estacionar();
    }

    private void rotaDireita() {
        moverFrente(0.5, 900);
        moverLado(-0.5, 600);
        autoAlignToGoal(ALVO_APRILTAG);
        disparar();
        estacionar();
    }

    private void rotaPadrao() {
        moverFrente(0.5, 700);
        autoAlignToGoal(ALVO_APRILTAG);
        estacionar();
    }

    private void estacionar() {
        girar(0.5, 450);
        moverTras(0.5, 600);
        elastico.setPower(0.6);
        sleep(400);
        elastico.setPower(0);
        pararMotores();
    }
    private static class ColorBlobProcessor implements VisionProcessor {
        private static final double LARGURA_REAL_CM = 6.0;
        private static final double FOCAL_LENGTH = 700.0;
        private static final Scalar HSV_MIN = new Scalar(20, 80, 80);
        private static final Scalar HSV_MAX = new Scalar(35, 255, 255);

        private final Mat hsv = new Mat();
        private final Mat mask = new Mat();
        private final List<MatOfPoint> contours = new ArrayList<>();
        private Rect maiorRetangulo;
        private int frameWidth = 0;
        private double distanciaCm = 0.0;
        private String posicao = "NENHUMA";

        @Override
        public void init(int width, int height, CameraCalibration calibration) {
            frameWidth = width;
        }

        @Override
        public Object processFrame(Mat input, long captureTimeNanos) {
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
            Core.inRange(hsv, HSV_MIN, HSV_MAX, mask);
            contours.clear();
            Imgproc.findContours(mask, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            maiorRetangulo = null;
            double maiorArea = 0.0;
            for (MatOfPoint contour : contours) {
                Rect rect = Imgproc.boundingRect(contour);
                double area = rect.area();
                if (area > maiorArea) {
                    maiorArea = area;
                    maiorRetangulo = rect;
                }
            }

            if (maiorRetangulo != null) {
                int centerX = maiorRetangulo.x + (maiorRetangulo.width / 2);
                if (centerX < frameWidth / 3) posicao = "ESQUERDA";
                else if (centerX > frameWidth * 2 / 3) posicao = "DIREITA";
                else posicao = "CENTRO";
                distanciaCm = (LARGURA_REAL_CM * FOCAL_LENGTH) / Math.max(maiorRetangulo.width, 1);
            } else {
                posicao = "NENHUMA";
                distanciaCm = 0.0;
            }

            return null;
        }

        @Override
        public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        }

        public String getPosicao() { return posicao; }
        public double getDistanciaCm() { return distanciaCm; }
    }
}
*/