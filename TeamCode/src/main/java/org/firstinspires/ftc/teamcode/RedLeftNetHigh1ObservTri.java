package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@Autonomous(name = "RedLeftNetHigh1ObservTri", group = "Autonomous")
public class RedLeftNetHigh1ObservTri extends LinearOpMode {

    //global variables
    public double liftPower = 0.1;
    public double highBasketPos = 3000;

    public double liftLevel1Ascent = 300;
    public double liftUpperLimit = 4000;
    public double liftLowerLimit = 50;

    // lift class
    public class Lift {
        public DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "vertLinArm");
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(liftPower);
                    initialized = true;
                }

                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < highBasketPos && pos < liftUpperLimit) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-liftPower);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0 && pos > liftLowerLimit) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }

        public class Level1Ascent implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(liftPower);
                    initialized = true;
                }

                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < liftLevel1Ascent && pos < liftUpperLimit) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }
        }

        public Action liftLevel1Ascent() {
            return new LiftUp();
        }

    }

    // claw class
    public static class Intake {
        private final Servo intake;

        public Intake(HardwareMap hardwareMap) {
            intake = hardwareMap.get(Servo.class, "intake");
        }

        public class Reset implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPosition(0.178);
                return false;
            }
        }
        public Action reset() {
            return new Reset();
        }

        public class Release implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                intake.setPosition(0.356);
                return false;
            }
        }
        public Action release() {
            return new Release();
        }

    }

    @Override
    public void runOpMode() {

        Pose2d initialPose = new Pose2d(60, -35, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // make a Claw instance
        Intake intake = new Intake(hardwareMap);
        // make a Lift instance
        Lift lift = new Lift(hardwareMap);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(53, 9, Math.toRadians(45)), Math.toRadians(90))
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(52, 53, Math.toRadians(45)), Math.toRadians(0));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(52, 53, Math.toRadians(45)))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(37, 0, Math.toRadians(180)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(60, -38), Math.toRadians(0));

        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        lift.liftUp(),
                        intake.release(),
                        lift.liftDown(),
                        tab2.build()
                )
        );
    }

}
