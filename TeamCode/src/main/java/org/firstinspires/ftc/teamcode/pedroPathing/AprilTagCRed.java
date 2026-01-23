
package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AprilTagCRed")
public class AprilTagCRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo raxon = hardwareMap.get(Servo.class, "raxon");
        Servo laxon = hardwareMap.get(Servo.class, "laxon");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "absEncoder");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        double pos = 0.48;
        double lastError = 0;
        raxon.setPosition(pos);
        laxon.setPosition(pos);

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                double error = result.getFiducialResults().get(0).getTargetXDegrees();
                double p = Math.abs(error);

                if (Math.abs(error) > 1.0) {
                    double derivative = error - lastError;
                    double correction = (error) - (derivative);

                    if (correction > 1) {
                        lastError = error;

                        pos += Math.max(-0.02, Math.min(0.02, correction));
                        pos = Math.max(0.19, Math.min(1.0, pos));
                        laxon.setPosition(pos);
                        raxon.setPosition(pos);
                    }

                    telemetry.addData("Angle", "%.0f°", (encoder.getVoltage() / 3.3) * 360);
                    telemetry.addData("error ", error);
                    telemetry.addData("derivation", derivative);
                    telemetry.addData("correction", correction);
                    telemetry.addData("position", pos);
                    telemetry.addData("raw error", p);

                    telemetry.update();
                }
            }

            sleep(25);
        }
    }
}