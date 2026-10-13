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

/**
 * TeleOp integrado da Lösungsjäger.
 *
 * ATENÇÃO: offsets e direções dos encoders do Pinpoint, direções dos motores
 * e posição neutra do servo devem ser confirmados no robô real antes de competir.
 * Motores de tração: REV HD Hex 41 (1291), com redução externa 5:1.
 * A redução não altera diretamente o comando de potência; cálculos de velocidade
 * e encoders devem considerar a redução e as especificações das rodas/encoders.
 */
@TeleOp(name = "Lösungsjäger - TeleOp Completo", group = "Lösungsjäger")
public class LJTeleOpCompleto extends LinearOpMode {

    // Limites conservadores iniciais; ajuste após testes de bancada.
    private static final double DRIVE_MAX = 0.70;
    private static final double DRIVE_PRECISION = 0.35;
    private static final double COLLECTOR_MAX = 0.50;
    private static final double LAUNCHER_MAX = 0.55;
    private static final double STICK_DEADBAND = 0.05;

    // Calibração do Pinpoint: substituir pelos offsets medidos no robô (mm).
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

        telemetry.addLine("Lösungsjäger TeleOp integrado pronto.");
        telemetry.addLine("Mantenha o robô imóvel durante a calibração inicial.");
        telemetry.addLine("Confirme direções dos motores, offsets e pods antes de dirigir.");
        telemetry.addData("Motor de tração", "REV HD Hex 41 (1291)");
        telemetry.addData("Redução externa", "5:1");
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
        // Padrão existente no Perry.java: esquerda invertida, direita normal.
        esq_f.setDirection(DcMotorSimple.Direction.REVERSE);
        esq_t.setDirection(DcMotorSimple.Direction.REVERSE);
        dir_f.setDirection(DcMotorSimple.Direction.FORWARD);
        dir_t.setDirection(DcMotorSimple.Direction.FORWARD);

        // Direções dos mecanismos são hipóteses iniciais; valide individualmente.
        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        lan1.setDirection(DcMotorSimple.Direction.FORWARD);
        lan2.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotorEx motor : new DcMotorEx[] {
                esq_f, esq_t, dir_f, dir_t, coletor, lan1, lan2
        }) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setPower(0.0);
        }

        // Não resetamos encoders dos motores: este TeleOp não usa seus ticks.
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

        // O robô deve estar parado ao zerar a orientação/pose do Pinpoint.
        pinpoint.resetPosAndIMU();

        // Servo posicional: posição inicial provisória; confira o mecanismo
        // e a folga mecânica antes de habilitar o servo no robô montado.
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
            // Pressione somente com o robô imóvel para redefinir a referência.
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

        // Mixagem mecanum; normalização mantém todas as potências em [-1, 1].
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
        // Gamepad 2: coletor independente, controle proporcional no stick esquerdo.
        double collectorPower = applyDeadband(-gamepad2.left_stick_y) * COLLECTOR_MAX;
        coletor.setPower(clip(collectorPower, -COLLECTOR_MAX, COLLECTOR_MAX));

        // Lançador 1: RT gira para frente; A solicita reversão.
        double launch1Power = 0.0;
        if (gamepad2.right_trigger > 0.05) {
            launch1Power = gamepad2.right_trigger * LAUNCHER_MAX;
        } else if (gamepad2.a) {
            launch1Power = -LAUNCHER_MAX;
        }
        lan1.setPower(clip(launch1Power, -LAUNCHER_MAX, LAUNCHER_MAX));

        // Lançador 2: LT gira para frente; B solicita reversão.
        double launch2Power = 0.0;
        if (gamepad2.left_trigger > 0.05) {
            launch2Power = gamepad2.left_trigger * LAUNCHER_MAX;
        } else if (gamepad2.b) {
            launch2Power = -LAUNCHER_MAX;
        }
        lan2.setPower(clip(launch2Power, -LAUNCHER_MAX, LAUNCHER_MAX));

        // Servo posicional ajustável em pequenos passos e limitado a [0, 1].
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
        telemetry.addData("Potência máxima rodas", "%.0f%%", DRIVE_MAX * 100.0);
        telemetry.addData("Precisão", gamepad1.left_bumper ? "ATIVA" : "inativa");
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("X", "%.1f mm", pose.getX(DistanceUnit.MM));
        telemetry.addData("Y", "%.1f mm", pose.getY(DistanceUnit.MM));
        telemetry.addData("Heading", "%.1f deg", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Coletor potência", "%.2f", coletor.getPower());
        telemetry.addData("Lançador 1 potência", "%.2f", lan1.getPower());
        telemetry.addData("Lançador 2 potência", "%.2f", lan2.getPower());
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
