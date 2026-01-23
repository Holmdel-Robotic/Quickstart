/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "AprilTagContinuous")
public class AprilTagContinuous extends LinearOpMode {

    @Override
    public void runOpMode() {
        Servo raxon = hardwareMap.get(Servo.class, "raxon");
        Servo laxon = hardwareMap.get(Servo.class, "laxon");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "absEncoder");
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(1);
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

                    if (correction >.5 && correction < 1) {
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