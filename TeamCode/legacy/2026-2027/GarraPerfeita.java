package org.firstinspires.ftc.teamcode;

//ServD - servo mão garra direito C1
//ServE - Servo mão garra esquerda C2
//ServC - Servo mão garra centro C0
//MotorGarra - motor que mexe o braço da garra E2

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class GarraPerfeita extends LinearOpMode {
    ElapsedTime runtime = new ElapsedTime();

    public DcMotor MotorGarra = null;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        MotorGarra = hardwareMap.get(DcMotor.class, "MotorGarra");
        CRServo ServD;
        ServD = hardwareMap.get(CRServo.class, "ServD");
        CRServo ServE;
        ServE = hardwareMap.get(CRServo.class, "ServE");
        CRServo ServC;
        ServC = hardwareMap.get(CRServo.class, "ServC");

        boolean flagserv;
        flagserv=false;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.y){
                //move to -135 degrees
                ServD.setPower(0);
                ServE.setPower(0);

            } else if (gamepad1.x || gamepad1.b) {
                //move to 0 degrees
                ServD.setPower(0.5);
                ServE.setPower(0.5);

            } else if (gamepad1.a) {
                //move to 135 degrees
                ServD.setPower(1);
                ServE.setPower(1);

            }

            if (gamepad2.x) //esse é para ele descer
                MotorGarra.setPower(30);
            else
                MotorGarra.setPower(0);

            if (gamepad2.b) //esse é para subir
                MotorGarra.setPower(-30);
            else
                MotorGarra.setPower(0);
        }
    }
}
