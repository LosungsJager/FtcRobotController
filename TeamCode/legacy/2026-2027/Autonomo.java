package org.firstinspires.ftc.teamcode;

import android.text.method.Touch;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class Autonomo extends LinearOpMode {
    public DcMotor esq_f, dir_f, esq_t, dir_t, baseGarra, movelevador, baseSobe, pendurar = null;
    BNO055IMU imu;
    double lim = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        esq_f = hardwareMap.get(DcMotor.class, "esq_f");
        esq_t = hardwareMap.get(DcMotor.class, "esq_t");
        dir_f = hardwareMap.get(DcMotor.class, "dir_f");
        dir_t = hardwareMap.get(DcMotor.class, "dir_t");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        baseGarra = hardwareMap.get(DcMotor.class, "base");
        movelevador = hardwareMap.get(DcMotor.class, "elevador");
        pendurar = hardwareMap.get(DcMotor.class, "pendurar");
        baseSobe = hardwareMap.get(DcMotor.class, "baseSobe");
        CRServo garraA;
        garraA = hardwareMap.get(CRServo.class, "garraA");
        CRServo garraB;
        garraB = hardwareMap.get(CRServo.class, "garraB");
        imu.initialize(parameters);
        DistanceSensor sensor1;
        sensor1 = hardwareMap.get(DistanceSensor.class, "sensor1");

//passo1 - locomover robô até a cesta
        double distancia = sensor1.getDistance(DistanceUnit.CM);
        while (lim < 0.6) {
            esq_f.setPower(-lim);//parado
            dir_f.setPower(-lim);
            esq_t.setPower(lim);
            dir_t.setPower(-lim + 0.15);
            lim = lim + 0.1;
        }
        distancia = sensor1.getDistance(DistanceUnit.CM);
        telemetry.addData("distance: ", distancia);
        telemetry.update();
        esq_f.setPower(0.22); //normal 0.3
        dir_f.setPower(0.3);
        esq_t.setPower(-0.3);
        dir_t.setPower(0.15);
        Thread.sleep(1950);

        esq_f.setPower(0);//parado
        dir_f.setPower(0);
        esq_t.setPower(0);
        dir_t.setPower(0);
        Thread.sleep(1000);

//passo 2 -

        pendurar.setPower(0.6);
        Thread.sleep(2600);
        pendurar.setPower(0);

        garraA.setPower(0);
        Thread.sleep(800);

        movelevador.setPower(5);
        Thread.sleep(5); // time sobe elevador
        Thread.sleep(2000);
        movelevador.setPower(0);
        Thread.sleep(20);

        garraA.setPower(0.29);
        Thread.sleep(10);
        garraB.setPower(-0.5);
        Thread.sleep(900);
        garraA.setPower(-0.75);
        Thread.sleep(1000);


        movelevador.setPower(-6000);
        Thread.sleep(600);

        //finaliza automato
        movelevador.setPower(-100); // retroceder elevador
        Thread.sleep(50);
        pendurar.setPower(-0.8); // baixar elevador
        Thread.sleep(2000);
        pendurar.setPower(0);
/*        esq_f.setPower(0.5*lim );// frente
        dir_f.setPower(0.5*lim);
        esq_t.setPower(-0.29*lim );
        dir_t.setPower(-0.5*lim);
        Thread.sleep(115);
  */

        }
    }
