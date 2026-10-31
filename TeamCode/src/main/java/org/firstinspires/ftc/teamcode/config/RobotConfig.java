package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/** Centralized hardware names and tuning constants. */
public final class RobotConfig {
    private RobotConfig() {}

    public static final class Hardware {
        private Hardware() {}
        public static final String LEFT_FRONT = "esq_f";
        public static final String LEFT_REAR = "esq_t";
        public static final String RIGHT_FRONT = "dir_f";
        public static final String RIGHT_REAR = "dir_t";
        public static final String INTAKE = "coletor";
        // Existing team code (Perry.java) uses the lower-case configuration name.
        public static final String PINPOINT = "pinpoint";
    }

    public static final class Drive {
        private Drive() {}
        // Inherited from Perry.java; validate with the drivetrain lifted.
        public static final DcMotorSimple.Direction LEFT_DIRECTION =
                DcMotorSimple.Direction.REVERSE;
        public static final DcMotorSimple.Direction RIGHT_DIRECTION =
                DcMotorSimple.Direction.FORWARD;
    }

    public static final class Intake {
        private Intake() {}
        public static final double IN_POWER = 1.0;
        public static final double OUT_POWER = -1.0;
        public static final double STOP_POWER = 0.0;
    }

    public static final class Motor {
        private Motor() {}
        // REV HD Hex 41 (1291) with an external 5:1 reduction.
        // Encoder-to-distance conversion is not enabled in the drivetrain yet.
        public static final double EXTERNAL_REDUCTION = 5.0;
    }
}
