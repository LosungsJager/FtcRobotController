package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

@Autonomous(name = "Auto Losungsjager - Decisão Tag")
public class ametista extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    int tagDetectada = -1; // -1 significa que nada foi visto ainda

    @Override
    public void runOpMode() {

        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);


        while (!isStarted() && !isStopRequested()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    tagDetectada = detection.id;
                }
            }

            telemetry.addData("Status", "Aguardando Start...");
            telemetry.addData("Tag Identificada", tagDetectada);
            telemetry.update();
        }


        visionPortal.close();

        if (tagDetectada == 1) {
            rotaEsquerda();
        } else if (tagDetectada == 2) {
            rotaCentro();
        } else if (tagDetectada == 3) {
            rotaDireita();
        } else {
            rotaPadrao();
        }
    }


    void rotaEsquerda() {
        telemetry.addLine("Executando Rota 1 (Esquerda)");
        telemetry.update();

        sleep(2000);
    }

    void rotaCentro() {
        telemetry.addLine("Executando Rota 2 (Centro)");
        telemetry.update();
        sleep(2000);
    }

    void rotaDireita() {
        telemetry.addLine("Executando Rota 3 (Direita)");
        telemetry.update();
        sleep(2000);
    }

    void rotaPadrao() {
        telemetry.addLine("Nenhuma tag vista. Indo para o estacionamento.");
        telemetry.update();
        sleep(2000);
    }
}
