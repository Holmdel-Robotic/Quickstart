package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.openftc.apriltag.AprilTagPose;

import java.util.List;


import java.util.function.Supplier;


@Configurable
@TeleOp


public class LimelightTest extends OpMode {

    private Limelight3A limelight;

    // Two Axon servos that rotate the turret together
    private Servo turretServoLeft;
    private Servo turretServoRight;


    // PID constants for turret control
    private double kP = 0.012;
    private double kI = 0.0;
    private double kD = 0.002;

    private double lastTime;

    private double sumError;
    private double prevError;


    // Current turret servo position
    private double turretPos = 0.5;

    @Override

    public void init() {

        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        turretServoLeft = hardwareMap.get(Servo.class, "laxon");
        turretServoRight = hardwareMap.get(Servo.class, "raxon");
        double prevError = 0;
        double sumError = 0;
        double lastTime = getRuntime();
    }

    @Override

    public void start() {

        // Configure Limelight
        limelight.pipelineSwitch(0);
        limelight.start();

    }

    @Override

    public void loop() {

            LLResult result = limelight.getLatestResult();

            boolean tag21Detected = false;
            double tx = 0;
            // Detect AprilTag with ID 21 and read its horizontal offset (tx)
            if (result != null && result.isValid() && result.getFiducialResults() != null) {
                    for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
                        if (tag.getFiducialId() == 21) {
                            tag21Detected = true;
                            tx = tag.getTargetXPixels();
                            break;
                        }
                    }
            }

            if (tag21Detected) {

                // Time tracking for PID calculations
                double now = getRuntime();
                double dt = Math.max(0.001, now - lastTime);
                lastTime = now;

                // PID error terms
                double error = tx;
                sumError += error * dt;
                double dError = (error - prevError) / dt;
                prevError = error;

                // PID output (unit: servo-position change)
                double output = kP * error + kI * sumError + kD * dError;

                // Convert PID output to a small servo adjustment
                double servoStep = output * 0.01;

                // Update turret position (invert to correct direction if needed)
                turretPos -= servoStep;

                // Limit turret position to valid servo range
                turretPos = Math.max(0.0, Math.min(1.0, turretPos));

                // Move both servos. One is reversed mechanically.
                turretServoLeft.setPosition(turretPos);
                turretServoRight.setPosition(1.0 - turretPos);

                // Shooter motors (example open-loop values)
                // laxon.setPower(0.95);
                // raxon.setPower(0.95);

                telemetry.addLine("Tracking AprilTag 21");
                telemetry.addData("tx", tx);
                telemetry.addData("Turret Servo Position", turretPos);

            } else {

                // No tag detected — turret holds its last known position
                telemetry.addLine("AprilTag 21 not detected");
            }

            telemetry.update();
//            idle();
        }

//        limelight.stop();
    }



