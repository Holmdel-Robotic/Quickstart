


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

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Configurable
@TeleOp(name = "AprilTagEasy")
public class AprilTagEasy extends LinearOpMode {

    private Limelight3A limelight;
    private Servo raxon;
    private Servo laxon;

    // Servo positions
    private double raxonPos = 0.4889;
    private double laxonPos = 0.4889;
    private static final double CENTER_POS = 0.4889;
    private static final double MIN_POS = 0.1894;
    private static final double MAX_POS = 0.7594;

    // Increased P gain for faster response
    private double kP = 0.008; // Increased from 0.003

    // Reduced smoothing for faster reaction
    private double lastError = 0;
    private static final double SMOOTHING = 0.3; // Reduced from 0.75 - less smooth but faster

    @Override
    public void runOpMode() {
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        raxon.setPosition(CENTER_POS);
        laxon.setPosition(CENTER_POS);

        waitForStart();

        while (opModeIsActive()) {
            track();
            telemetry.update();
            sleep(10); // speed up for fast updating
        }

        limelight.close();
    }

    private void track() {
        LLResult result = limelight.getLatestResult();

        // No target? Do nothing
        if (result == null || !result.isValid()) {
            telemetry.addData("Status", "No target");
            lastError = 0;
            return;
        }

        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        if (tags.isEmpty()) {
            telemetry.addData("Status", "No AprilTags");
            lastError = 0;
            return;
        }

        // Get error from first tag
        double rawError = tags.get(0).getTargetXDegrees();

        //blend with last error
        double error = (SMOOTHING * lastError) + ((1 - SMOOTHING) * rawError);
        lastError = error;

        //initial response
        if (Math.abs(error) < 0.3) { // Reduced from 0.5
            error = 0;
        }

        // Calculate correction
        double correction = kP * error;
        correction = Math.max(-0.15, Math.min(0.15, correction)); // Increased limit from 0.05 to 0.15

        // Move servos
        laxonPos = CENTER_POS + correction;
        raxonPos = CENTER_POS + correction;

        // Keep in bounds
        laxonPos = Math.max(MIN_POS, Math.min(MAX_POS, laxonPos));
        raxonPos = Math.max(MIN_POS, Math.min(MAX_POS, raxonPos));

        if(raxonPos > 1)
        {
            raxonPos = 1;
        }
        if(raxonPos < .1894)
        {
            raxonPos = .1894;
        }
        if(laxonPos < .1894)
        {
            laxonPos = .1894;
        }
        if(laxonPos > 1)
        {
            laxonPos = 1;
        }

        laxon.setPosition(laxonPos);
        raxon.setPosition(raxonPos);

        // Show what's happening
        telemetry.addData("Target ID", tags.get(0).getFiducialId());
        telemetry.addData("Raw Error", "%.2f deg", rawError);
        telemetry.addData("Smoothed", "%.2f deg", error);
        telemetry.addData("Correction", "%.4f", correction);

        if (Math.abs(error) < 0.3) {
            telemetry.addLine("LOCKED ON");
        }
    }
}

