package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "DECODE2", group = "Competition")
public class DECODE2 extends LinearOpMode {

    private DcMotor launcher, escovas;
    private Servo rampa, cano;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private static final double POS_ENTRADA = 0.2;
    private static final double POS_TRAVAR_RAMPA = 0.5;
    private static final double POS_LANCAR = 0.8;
    private static final double CANO_ABERTO = 0.0;
    private static final double CANO_FECHADO = 0.3;

    private int tagID = -1;

    @Override
    public void runOpMode() {

        launcher = hardwareMap.get(DcMotor.class, "launcher");
        escovas = hardwareMap.get(DcMotor.class, "escovas");
        rampa = hardwareMap.get(Servo.class, "rampa");
        cano = hardwareMap.get(Servo.class, "cano");

        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(DcMotor.Direction.FORWARD);
        escovas.setDirection(DcMotor.Direction.FORWARD);

        rampa.setPosition(POS_TRAVAR_RAMPA);
        cano.setPosition(CANO_ABERTO);

        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();

        telemetry.addLine("Inicializando visão...");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();
            if (detections.size() > 0) {
                tagID = detections.get(0).id;
                telemetry.addData("Tag Detectado", tagID);
                if (tagID == 21) telemetry.addLine("Sequência: GPP");
                else if (tagID == 22) telemetry.addLine("Sequência: PGP");
                else if (tagID == 23) telemetry.addLine("Sequência: PPG");
                else telemetry.addLine("Tag desconhecido");
            } else telemetry.addLine("Nenhum AprilTag detectado.");
            telemetry.update();
            sleep(50);
        }

        waitForStart();

        if (opModeIsActive()) {
            visionPortal.stopStreaming();
            executarSequencia(tagID);
        }

        visionPortal.close();
    }

    private void executarSequencia(int id) {
        telemetry.addData("Executando sequência", id);
        telemetry.update();

        launcher.setPower(1.0);
        sleep(1500);

        String sequencia;
        if (id == 21) sequencia = "GPP";
        else if (id == 22) sequencia = "PGP";
        else if (id == 23) sequencia = "PPG";
        else sequencia = "GPP";

        for (char cor : sequencia.toCharArray()) {
            escovas.setPower(0.6);
            rampa.setPosition(POS_ENTRADA);
            sleep(500);
            empurrarBola();
            rampa.setPosition(POS_TRAVAR_RAMPA);
            escovas.setPower(0);
            sleep(700);
        }

        launcher.setPower(0);
        rampa.setPosition(POS_TRAVAR_RAMPA);
        cano.setPosition(CANO_FECHADO);
    }

    private void empurrarBola() {
        cano.setPosition(CANO_FECHADO);
        sleep(300);
        cano.setPosition(CANO_ABERTO);
    }
}
