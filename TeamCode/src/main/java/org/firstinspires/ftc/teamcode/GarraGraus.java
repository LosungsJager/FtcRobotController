package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp

public class GarraGraus extends LinearOpMode {
    private Servo ServD, ServE;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ServD = hardwareMap.get(Servo.class, "ServD");
        ServE = hardwareMap.get(Servo.class, "ServE");

        ServD.setPosition(0);
        ServE.setPosition(0);


        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.y) {
                //move to position 0
                ServD.setPosition(0);
                ServE.setPosition(0);

            } else if (gamepad1.x || gamepad1.b) {
                //move to position 0.5
                ServD.setPosition(0.5);
                ServE.setPosition(0.5);

            } else if (gamepad1.a) {
                //move to position 1
                ServD.setPosition(1);
                ServE.setPosition(1);
            }
            telemetry.addData("Servo Position", ServD.getPosition());
            telemetry.addData("Servo Position", ServE.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
