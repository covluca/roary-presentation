package org.firstinspires.ftc.teamcode.RoaryX7_Decode.opmode.hardware;


import com.acmerobotics.dashboard.config.Config;

@Config
public class Globals {
    public static class Kirby {
        public static double TICKS_PER_ROTATION_PER_MINUTE_PRIMARY = 28.0 / 60.0;
        public static double TICKS_PER_DEGREE_TURRET = 12.15;
        public static double TURRET_MAX_DEG_PER_SEC = 217.2;
        public static double MID_DOWN = 0.03;
        public static double MID_FORCED_DOWN = 0;
        public static double MID_UP = 0.24;
        public static double MID_LAUNCH = 0.3;
        public static double MID_RESEVED = 0.08;
        public static double LEFT_DOWN = 0.05;
        public static double LEFT_FORCED_DOWN = 0;
        public static double LEFT_UP = 0.3;
        public static double LEFT_LAUNCH = 0.35;
        public static double LEFT_RESEVED = 0.08;

        public static double RIGHT_DOWN = 0.32;
        public static double RIGHT_FORCED_DOWN = 0.28;
        public static double RIGHT_UP = 0.8;
        public static double RIGHT_LAUNCH = 0.85;
        public static double RIGHT_RESEVED = 0.48;

        public static double TURRET_LEFT_DEG = -58;
        public static double TURRET_STRAIGHT_DEG = 0;
        public static double TURRET_RIGHT_DEG = 47;
        public static double ELEVATION_MIN = 0.22;
        public static double ELEVATION_MAX = 1;
        public static double ELEVATION_MIN_DEG = 33;
        public static double ELEVATION_MAX_DEG = 60.2;
        public static double RPM_MIN = 1300;
        public static double RPM_MAX = 3800;
        public static double REAL_RPM_MAX = 4500;
        public static double MPSCOMPENSATOR = 2.25;

        public static double DELTA_RPM = 210;

        public static class Codes {
            public static double OFF = 0;
            public static double RED = 0.283;
            public static double ORANGE = 0.333;
            public static double YELLOW = 0.388;
            public static double GREEN = 0.5;
            public static double BLUE = 0.61;
            public static double INDIGO = 0.66;
            public static double PURPLE = 0.71;
            public static double WHITE = 1;
        }
    }

    public static class Sasha {
        public static final double TICKS_PER_ROTATION_PER_MINUTE_SECONDARY = 103.8 / 60.0;
        public static final double TICKS_PER_ROTATION_PER_MINUTE_PRIMARY = 28.0 / 60.0;
        public static final double TICKS_PER_DEGREE = 1121.0 / 90.0;
        public static final double TURRET_OFSET = 1361 / TICKS_PER_DEGREE - 3;

        public static double ELEVATION_MIN = 0.03;
        public static double ELEVATION_MAX = 0.48;

        public static double ELEVATION_MIN_DEG = 33;
        public static double ELEVATION_MAX_DEG = 52;

        public static double RPM_MIN = 2850;
        public static double RPM_MAX = 4500;

        public static double MPSCOMPENSATOR = 2.82;
        public static double TURRET_OFFSET = 8;


        //Intake
        public static double TAVALUG_LOAD = 0.47;
        public static double TAVALUG_UNLOAD = 0.81; //0.81
        public static double TAVALUG_HOLD = 0.57;
        public static double INDEXER_S5 = 1;
        public static double INDEXER_L4 = 0.88;
        public static double INDEXER_S4 = 0.75;
        public static double INDEXER_L3 = 0.63;
        public static double INDEXER_S3 = 0.5;
        public static double INDEXER_L2 = 0.38;
        public static double INDEXER_S2 = 0.25;
        public static double INDEXER_L1 = 0.13;
        public static double INDEXER_S1 = 0;
    }

    public static class Kathy {
        public static double TAVALUG_LOAD = 0.21;
        public static double TAVALUG_UNLOAD = 0.03;
        public static double TAVALUG_HOLD = 0.13;

        public static double INDEXER_S1 = 1;
        public static double INDEXER_L1 = 0.85;
        public static double INDEXER_S2 = 0.75;
        public static double INDEXER_L2 = 0.6;
        public static double INDEXER_S3 = 0.5;
        public static double INDEXER_L3 = 0.35;
        public static double INDEXER_S4 = 0.25;
        public static double INDEXER_L4 = 0.1;
        public static double INDEXER_S5 = 0;


        public static double TURRET_LEFT = 0.98;
        public static double TURRET_STRAIGHT = 0.48;
        public static double TURRET_RIGHT = 0;

        public static double TICK_PER_DEGREE_TURRET = 0.0117;

        public static double TURRET_LEFT_DEG = -58;
        public static double TURRET_STRAIGHT_DEG = 0;
        public static double TURRET_RIGHT_DEG = 47;

        public static double ELEVATION_MIN = 0;
        public static double ELEVATION_MAX = 0.48;

        public static double ELEVATION_MIN_DEG = 33;
        public static double ELEVATION_MAX_DEG = 52;

        public static double RPM_MIN = 2750;
        public static double RPM_MAX = 4500;

        public static double MPSCOMPENSATOR = 2.83;
//        public static double MPSCOMPENSATOR = 3.0932;

    }

    public static class Octav {
        static final double FEET_PER_METER = 3.28084;
        public static double DEAD_WHEEL_RAD = 3;

        // field shit
        public static double ROTACASTER_RAD = 1.75;

        //video
        public static double ENCODER_TICKS = 8192;
        public static double TICKS_PER_CM_DRIVE = ENCODER_TICKS / (2 * Math.PI * DEAD_WHEEL_RAD);
        public static double ROTA_TICKS_CM = ENCODER_TICKS / (2 * Math.PI * ROTACASTER_RAD);
        //public static double SPOOL_RADIUS = 2.7;
        //public static double LIFT_MOTOR_TICKS = 384.5;
        //public static double TICKS_PER_CM_LIFT = LIFT_MOTOR_TICKS / (2 * Math.PI * SPOOL_RADIUS);

        public static double HEXT_UNLOAD = 0.95;
        public static double HEXT_LOAD = 0.7;
        public static double HEXT_UNLOAD_CM = -1;
        public static double HEXT_LOAD_CM = 38.5;

        public static double CLAW_CLOSED = 0.85;
        public static double CLAW_OPEN = 0.38;
        public static double CLAW_OPEN_IN_SUBMERSIBLE = 0.64;
        public static double CLAW_OPEN_IN_ROBOT = 0.75;
        public static double CLAW_OPEN_IN_ROBOT_MORE = 0.64;

        public static double OUTTAKE_CLAW_CLOSED = 0.98;
        public static double OUTTAKE_CLAW_OPEN = 0.7;

        public static double ROTATE_CLAW_MAX = 0.89;
        public static double ROTATE_CLAW_MID = 0.5;
        public static double ROTATE_CLAW_LEFT = 0.7;
        public static double ROTATE_CLAW_RIGHT = 0.3;
        public static double ROTATE_CLAW_MIN = 0.17;

        public static double ELBOW_LOAD_SPECIMEN = 0;
        public static double ELBOW_LOAD = 0.24;
        public static double ELBOW_PREP_LOAD = 0.36;
        public static double ELBOW_UNLOAD = 0.64;
        public static double ELBOW_CARRY = 0.48;

        public static double WRIST_LOAD_SPECIMEN_HOME = 0.82;
        public static double WRIST_LOAD_SPECIMEN_FAR = 0.87;
        public static double WRIST_LOAD = 0.13;
        public static double WRIST_UNLOAD = 0.93;
        public static double WRIST_CARRY = 0.75;

        public static double BASC_LOAD = 0.45;
        public static double BASC_UNLOAD_SPECIMEN = 0.03;
        public static double BASC_UNLOAD = 0.17;

        public static double LIFT_LOAD = 0;
        public static double LIFT_MIN_BASC_HEIGHT = 17;
        public static double LIFT_RAISE = 22;
        public static double LIFT_CLIMB = 44;
        public static double LIFT_SPECIMEN = 48;
        public static double LIFT_SPECIMEN_SECURE = 58;
        public static double LIFT_MAX = 93;
        public static double LIFT_UNLOAD = 90.8;

        public static double PLANE_LOAD = 0;
        public static double PLANE_LAUNCH = 0.8;

        public static double PITCH_IN = 0;
        public static double PITCH_OUT = 0.24;

        public static double CLAW_SPECIMEN_CLOSED = 0.93;
        public static double CLAW_SPECIMEN_OPEN = 0.77;

        public static double BASC_SPECIMEN_UNLOAD = 0.4;
        public static double BASC_SPECIMEN_CLIP = 0.64;
        public static double BASC_SPECIMEN_LOAD = 0.85;
    }

    public static class Macara {
        //public static double SPOOL_RADIUS = 2.7;
        //public static double LIFT_MOTOR_TICKS = 384.5;
        //public static double TICKS_PER_CM_LIFT = LIFT_MOTOR_TICKS / (2 * Math.PI * SPOOL_RADIUS);

        public static double CLAW_CLOSED = 0.95;
        public static double CLAW_OPEN = 0.55;

        public static double CLAW_OPEN_IN_SUBMERSIBLE = 0.64;
        public static double CLAW_OPEN_IN_ROBOT = 0.75;
        public static double CLAW_OPEN_IN_ROBOT_MORE = 0.64;

        public static double ROTATE_CLAW_MAX = 0.85;
        public static double ROTATE_CLAW_RIGHT = 0.73;
        public static double ROTATE_CLAW_MID = 0.57;
        public static double ROTATE_CLAW_LEFT = 0.43;
        public static double ROTATE_CLAW_MIN = 0.29;
        public static double ROTATE_CLAW_SPECIMEN = 0;

        //public static double WRIST_LOAD_SPECIMEN_HOME = 0.82;
        //public static double WRIST_LOAD_SPECIMEN_FAR = 0.87;
        public static double WRIST_PREP_LOAD = 1;
        public static double WRIST_LOAD = 0.8;
        public static double WRIST_UNLOAD = 0;
        /// public static double WRIST_UNLOAD = 0.08;
        public static double WRIST_UNLOAD_LIFT = 0.3;
        public static double WRIST_CARRY = 0.64;
        public static double WRIST_SCAN = 0.43;
        public static double WRIST_SPECIMEN = 0.64;

        //public static double ELBOW_LOAD_SPECIMEN = 0;
        public static double ELBOW_LOAD = 0.49;
        public static double ELBOW_LOAD_CLOSE = 0.47;
        public static double ELBOW_PREP_LOAD = 0.71;
        public static double ELBOW_UNLOAD = 0.92;
        public static double ELBOW_CARRY = 0.75;

        /*

        INAINTE DE LOVITURA

        public static double ELBOW_LOAD = 0.55;
        public static double ELBOW_LOAD_CLOSE = 0.52;
        public static double ELBOW_PREP_LOAD = 0.76;
        public static double ELBOW_UNLOAD = 0.96;
        public static double ELBOW_CARRY = 0.8;
        */

        public static double HEXT_UNLOAD_CM = 0;
        public static double HEXT_LOAD_CM = 37;

        public static double OUTTAKE_CLAW_CLOSED = 0.98;
        public static double OUTTAKE_CLAW_SECURE = 1;
        public static double OUTTAKE_CLAW_SPECIMEN = 0.94;
        public static double OUTTAKE_CLAW_ANTISHOCK = 0.89;
        public static double OUTTAKE_CLAW_OPEN_IN_BASKET = 0.8;
        public static double OUTTAKE_CLAW_OPEN = 0.64;

        /// LA 0.79 RATA TRANSFERE
        /// LA 0.78 ERA OK-UT DAR TOT MAI TREBUIE
        /*
        public static double BASC_LOAD = 0.76;
        public static double BASC_ANTI_SHOCK = 0.55;
        public static double BASC_UNLOAD_SPECIMEN = 0.44;
        public static double BASC_UNLOAD = 0.35;
        public static double BASC_UNLOAD_LOW = 0.31;
        public static double BASC_UNLOAD_ANTI_SHOCK = 0.27;
         */

        public static double BASC_LOAD = 0.251;

        public static double BASC_UNLOAD_SPECIMEN_DEVIANT = 0.4;
        public static double BASC_SECURE_SPECIMEN_DEVIANT = 0.48;

        public static double BASC_ANTI_SHOCK = 0.53;

        /// TODO AAAAH THIS POSITION IS A LITTLE BIT ABOVE THE CHAMBER AND IT WON'T WORK PLACING SPECIMENS MANUALLY
        /// TODO RIGHT POSITION IS 0.62 RAAAAH
        public static double BASC_UNLOAD_HOLD_SPECIMEN = 0.58;
        public static double BASC_UNLOAD_SPECIMEN = 0.62;

        public static double BASC_UNLOAD = 0.65;
        public static double BASC_UNLOAD_LOW = 0.69;
        public static double BASC_UNLOAD_ANTI_SHOCK = 0.73;
        public static double BASC_SECURE_SPECIMEN_PLEASE = 0.8;
        public static double BASC_UNLOAD_LIMIT = 0.8;

        public static double RAIL_AT_INTAKE = 1;
        public static double RAIL_LOAD = 0.55;
        public static double RAIL_AT_OUTTAKE = 0;

        public static double RAIL_MIN_CM = 0;
        public static double RAIL_LOAD_CM = 6;
        public static double RAIL_MAX_CM = 12;


        public static double LIFT_LOAD = 43;
        public static double LIFT_UNLOAD_LOW_BASKET = 64.7;
        public static double LIFT_UNLOAD_TRAVEL = 71;
        //        public static double LIFT_MIN_BASC_HEIGHT = 17;
//        public static double LIFT_RAISE = 22;
//        public static double LIFT_CLIMB = 44;
//        public static double LIFT_SPECIMEN = 48;
//        public static double LIFT_SPECIMEN_SECURE = 58;
        public static double LIFT_MAX = 116;
        public static double LIFT_UNLOAD = 107.5;
        public static double LIFT_UNLOAD_SPECIMEN = 48;

    }

    public static class Echo {
        //claw intake
        public static double CLAW_CLOSED = 0.07;   // greu de ghicit
        public static double CLAW_OPEN_MIN = 0.237; // pt apropriere
        public static double CLAW_OPEN_MID = 0.32; // pt transfer si folosire generala
        public static double CLAW_OPEN_MAX = 0.46; // pt impingere si mutare
//
//        //Differential
//        public static double GD_UNLOAD = 1;
//        public static double GS_UNLOAD = 0;
//        public static double GD_LOAD = 0.42;
//        public static double GS_LOAD = 1 - GD_LOAD;
//        public static double GD_UNLOAD_INTER = 0.9; // momentan ochiometrice
//        public static double GS_UNLOAD_INTER = 0.1;
//        public static double GD_LOAD_INTER = 0.75;
//        public static double GS_LOAD_INTER = 0.35;


        public static double GD_UNLOAD = 0.16;
        public static double GS_UNLOAD = GD_UNLOAD;
        public static double GD_LOAD = 0.63;
        public static double GS_LOAD = GD_LOAD;
        public static double GD_UNLOAD_INTER = 0.3; // momentan ochiometrice
        public static double GS_UNLOAD_INTER = GD_UNLOAD_INTER;
        public static double GD_LOAD_INTER = 0.54;
        public static double GS_LOAD_INTER = GD_LOAD_INTER;

        //extensie intake
        public static double HEXT_LOAD = 0.76; //0.73
        public static double HEXT_UNLOAD = 0.965;
        public static double HEXT_LOAD_CM = 45;
        public static double HEXT_UNLOAD_CM = -0.5;


        public static double LIFT_MAX_HEIGHT = 95; // teoretic 100
        public static double LIFT_UNLOAD = 90; // teoretic 100
        public static double LIFT_HEIGHT_SPECIMEN = 36.45;
        public static double LIFT_HEIGHT_SPECIMEN_PLACE = 51.50;

        public static double DISTANCE_F_SPECIMEN = 12d;
        public static double DISTANCE_F_SPECIMEN_MIN = 11.5;
        public static double DISTANCE_I_SAFE_TRANSFER = 25.0;


        public static double SWEEP_HIDE = 0.02;
        public static double SWEEP_SWEEP = 0.3;

        public static double CLAW_OUT_CLOSE = 0.163;
        public static double CLAW_OUT_OPEN = 0.39;

        public static double BASCULA_LOAD = 0;
        public static double BASCULA_INTER = 0.41;
        public static double BASCULA_UNLOAD = 0.68;
        public static double BASCULA_INTER_UNLOAD = (BASCULA_INTER + BASCULA_UNLOAD) / 2;


        public static double HEXT_HELP_LOAD = 0.55;
        public static double HEXT_HELP_UNLOAD = 0.97;
        public static double HEXT_HELP_INTER = (HEXT_HELP_LOAD + HEXT_HELP_UNLOAD) * 0.5;

        public static double BASC_SPECIMEN_UNLOAD = 0.36;
        public static double BASC_SPECIMEN_CLIP = 0.55;
        public static double BASC_SPECIMEN_LOAD = 0.85;
    }

    public static class Clippy {
        public static String WRIST_STRING = "WRIST";
        public static String CLAW_STRING = "C2";
        public static String OUT_EXT_STRING = "C1";
        public static String BASC_STRING = "C0";

        public static double CLAW_CLOSED = 0.06;
        public static double CLAW_OPEN = 0.14;
        public static double CLAW_OPEN_WIDE = 0.21;

        public static double WRIST_LOAD = 0.91;
        public static double WRIST_UNLOAD = 0.448;
        public static double WRIST_INTER = 0.66;

        public static double INTAKE_LOAD = 1;
        public static double INATKE_UNLOAD = -0.5;
        public static double INTAKE_STOP = 0;
        public static double INTAKE_OFFSET = 0.9;

        public static double BASC_LOAD = 0;
        public static double BASC_UPRIGHT = 0.28;
        public static double BASC_UNLOAD_CLIP = 0.42;
        public static double BASC_UNLOAD_SAMPLE = 0.32;
        public static double BASC_LOAD_CLIP = 0.635;
        public static double BASC_PREP_CLIP = 0.2;
        public static double BASC_DRIVE = 0.52;


        public static double OEXT_HOME = 0.92;
        public static double OEXT_AWAY = 0.53;
        public static double OEXT_CLIP_LOAD = 0.72;
        public static double OEXT_JUST_A_BIT = 0.85;

        public static double HEXT_MAX_CM = 37;
        public static double HEXT_MIN_CM = 0;

        public static double LIFT_NO_POWER_NEEDED = 48;
        public static double MAX_LIFT_HEIGHT = 67.3;
        public static double HIGH_BASKET_LIFT_HEIGHT = 56;

        public static double POLAR_TICKS_TO_90 = 192;
        public static double TICKS_PER_DEGREES = POLAR_TICKS_TO_90 / 8;

        public static double DISTANCE_F_SAFE_TRANSFER = 23;
        public static double DISTANCE_F_PLACE_SPECIMEN = 2.3;
    }
}
