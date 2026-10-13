package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "BIOBUZZ2027", group = "Lösungsjäger")
public class LJTeleOpCompleto extends LinearOpMode {

    private static final double DRIVE_MAX = 0.70;
    private static final double DRIVE_PRECISION = 0.35;
    private static final double COLLECTOR_MAX = 0.50;
    private static final double LAUNCHER_MAX = 0.55;
    private static final double STICK_DEADBAND = 0.05;

    // medir no robo
    private static final double PINPOINT_X_OFFSET_MM = 0.0;
    private static final double PINPOINT_Y_OFFSET_MM = 0.0;

    private static final double SERVO_START_POSITION = 0.50;
    private static final double SERVO_STEP = 0.02;

    private DcMotorEx esq_f;
    private DcMotorEx esq_t;
    private DcMotorEx dir_f;
    private DcMotorEx dir_t;
    private DcMotorEx coletor;
    private DcMotorEx lan1;
    private DcMotorEx lan2;
    private Servo serv;
    private GoBildaPinpointDriver pinpoint;

    private boolean fieldCentric = false;
    private boolean previousY = false;
    private boolean previousOptions = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;
    private boolean previousX = false;
    private double servoPosition = SERVO_START_POSITION;

    @Override
    public void runOpMode() {
        mapHardware();
        configureHardware();

        telemetry.addLine("TeleOp integrado pronto.");
        telemetry.addLine("pera.");
        telemetry.addLine("ve se ta certo.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopAllMotors();
            return;
        }

        while (opModeIsActive()) {
            pinpoint.update();

            updateDriveModeAndReset();
            driveMecanum();
            controlMechanisms();
            updateTelemetry();

            idle();
        }

        stopAllMotors();
    }

    private void mapHardware() {
        esq_f = hardwareMap.get(DcMotorEx.class, "esq_f");
        esq_t = hardwareMap.get(DcMotorEx.class, "esq_t");
        dir_f = hardwareMap.get(DcMotorEx.class, "dir_f");
        dir_t = hardwareMap.get(DcMotorEx.class, "dir_t");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        coletor = hardwareMap.get(DcMotorEx.class, "coletor");
        lan1 = hardwareMap.get(DcMotorEx.class, "lan1");
        lan2 = hardwareMap.get(DcMotorEx.class, "lan2");
        serv = hardwareMap.get(Servo.class, "serv");
    }

    private void configureHardware() {
        esq_f.setDirection(DcMotorSimple.Direction.REVERSE);
        esq_t.setDirection(DcMotorSimple.Direction.REVERSE);
        dir_f.setDirection(DcMotorSimple.Direction.FORWARD);
        dir_t.setDirection(DcMotorSimple.Direction.FORWARD);

        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        lan1.setDirection(DcMotorSimple.Direction.FORWARD);
        lan2.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[] {
                esq_f, esq_t, dir_f, dir_t, coletor, lan1, lan2
        }) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        }

        pinpoint.setOffsets(
                PINPOINT_X_OFFSET_MM,
                PINPOINT_Y_OFFSET_MM,
                DistanceUnit.MM
        );
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        // deve ta errado
        servoPosition = clip(servoPosition, 0.0, 1.0);
        serv.setPosition(servoPosition);
    }

    private void updateDriveModeAndReset() {
        boolean yNow = gamepad1.y;
        if (yNow && !previousY) {
            fieldCentric = !fieldCentric;
        }
        previousY = yNow;

        boolean optionsNow = gamepad1.options;
        if (optionsNow && !previousOptions) {
            // so se o robo ta parado
            pinpoint.resetPosAndIMU();
        }
        previousOptions = optionsNow;
    }

    private void driveMecanum() {
        double forward = applyDeadband(-gamepad1.left_stick_y);
        double strafe = applyDeadband(gamepad1.left_stick_x);
        double turn = applyDeadband(gamepad1.right_stick_x);

        double heading = pinpoint.getHeading(AngleUnit.RADIANS);
        if (fieldCentric) {
            double cos = Math.cos(-heading);
            double sin = Math.sin(-heading);
            double fieldStrafe = strafe * cos - forward * sin;
            double fieldForward = strafe * sin + forward * cos;
            strafe = fieldStrafe;
            forward = fieldForward;
        }

        // manter em [-1, 1].
        double pEsqF = forward + strafe + turn;
        double pEsqT = forward - strafe + turn;
        double pDirF = forward - strafe - turn;
        double pDirT = forward + strafe - turn;

        double denominator = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(pEsqF), Math.abs(pEsqT)),
                        Math.max(Math.abs(pDirF), Math.abs(pDirT))
                )
        );

        double max = gamepad1.left_bumper
                ? DRIVE_MAX * DRIVE_PRECISION
                : DRIVE_MAX;

        esq_f.setPower(clip((pEsqF / denominator) * max, -DRIVE_MAX, DRIVE_MAX));
        esq_t.setPower(clip((pEsqT / denominator) * max, -DRIVE_MAX, DRIVE_MAX));
        dir_f.setPower(clip((pDirF / denominator) * max, -DRIVE_MAX, DRIVE_MAX));
        dir_t.setPower(clip((pDirT / denominator) * max, -DRIVE_MAX, DRIVE_MAX));
    }

    private void controlMechanisms() {
        double collectorPower = applyDeadband(-gamepad2.left_stick_y) * COLLECTOR_MAX;
        coletor.setPower(clip(collectorPower, -COLLECTOR_MAX, COLLECTOR_MAX));

        double launch1Power = 0.0;
        if (gamepad2.right_trigger > 0.05) {
            launch1Power = gamepad2.right_trigger * LAUNCHER_MAX;
        } else if (gamepad2.a) {
            launch1Power = -LAUNCHER_MAX;
        }
        lan1.setPower(clip(launch1Power, -LAUNCHER_MAX, LAUNCHER_MAX));

        double launch2Power = 0.0;
        if (gamepad2.left_trigger > 0.05) {
            launch2Power = gamepad2.left_trigger * LAUNCHER_MAX;
        } else if (gamepad2.b) {
            launch2Power = -LAUNCHER_MAX;
        }
        lan2.setPower(clip(launch2Power, -LAUNCHER_MAX, LAUNCHER_MAX));

        //seervo [0, 1].
        boolean dpadLeftNow = gamepad2.dpad_left;
        boolean dpadRightNow = gamepad2.dpad_right;
        boolean xNow = gamepad2.x;

        if (dpadLeftNow && !previousDpadLeft) {
            servoPosition -= SERVO_STEP;
        }
        if (dpadRightNow && !previousDpadRight) {
            servoPosition += SERVO_STEP;
        }
        if (xNow && !previousX) {
            servoPosition = SERVO_START_POSITION;
        }

        servoPosition = clip(servoPosition, 0.0, 1.0);
        serv.setPosition(servoPosition);

        previousDpadLeft = dpadLeftNow;
        previousDpadRight = dpadRightNow;
        previousX = xNow;
    }

    private void updateTelemetry() {
        Pose2D pose = pinpoint.getPosition();

        telemetry.addData("Modo de direção", fieldCentric ? "Field-centric" : "Robot-centric");
        telemetry.addData("Pot. rodas", "%.0f%%", DRIVE_MAX * 100.0);
        telemetry.addData("Precisão", gamepad1.left_bumper ? "ATIVA" : "inativa");
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("X", "%.1f mm", pose.getX(DistanceUnit.MM));
        telemetry.addData("Y", "%.1f mm", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading", "%.1f deg", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Coletor potência", "%.2f", coletor.getPower());
        telemetry.addData("Lan.1 pot.", "%.2f", lan1.getPower());
        telemetry.addData("Lan.2 pot.", "%.2f", lan2.getPower());
        telemetry.addData("Servo posição", "%.2f", servoPosition);
        telemetry.addLine("G1: sticks=movimento, Y=field-centric, LB=precisão, OPTIONS=reset Pinpoint");
        telemetry.addLine("G2: LS=coletor, RT/A=lan1, LT/B=lan2, D-pad L/R=servo, X=servo centro");
        telemetry.update();
    }

    private void stopAllMotors() {
        if (esq_f != null) esq_f.setPower(0.0);
        if (esq_t != null) esq_t.setPower(0.0);
        if (dir_f != null) dir_f.setPower(0.0);
        if (dir_t != null) dir_t.setPower(0.0);
        if (coletor != null) coletor.setPower(0.0);
        if (lan1 != null) lan1.setPower(0.0);
        if (lan2 != null) lan2.setPower(0.0);
    }

    private double applyDeadband(double value) {
        return Math.abs(value) < STICK_DEADBAND ? 0.0 : value;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
