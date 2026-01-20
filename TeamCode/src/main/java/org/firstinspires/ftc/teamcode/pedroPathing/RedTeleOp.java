
package org.firstinspires.ftc.teamcode.pedroPathing;
import android.net.EthernetNetworkSpecifier;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorMROpticalDistance;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class RedTeleOp extends OpMode {
    private Follower follower;

    private boolean debounceA;
    private Timer pathTimer;

    private  Timer actiontimer;

    private  Timer timerA;

    private Servo raxon;

    private  Servo gate;
    private Servo laxon;
    private Servo blocker;
    private Servo hood;

    private double x;
    private double y;

    double ballsPassed;
    private double distance;
    private boolean debounceB;

    private boolean debounceX;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;

    private boolean autoTarget = false;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx intakeOuter;
    private boolean intakeOn;
    private boolean flywheelOn;

    private IMU imu;
    private boolean kickerpos;

    private boolean debounce_dpad_up;
    private boolean debounce_dpad_down;
    private boolean debounceY;

    private boolean debounceDL, debounceDR, debounceLB, debounceRB;

    private  boolean debounceBACK;

    private boolean debounceStart;
    private boolean debounceGUIDE;

    private boolean debounceLEFT_TRIGGER;
    private boolean debounceRIGHT_TRIGGER;

    private double flywheelVelocity;
    private boolean feederOn;
    private DcMotorEx intakeInner;

    private DistanceSensor distanceSensor;
    private CRServo feederL;

    private CRServo feederR;
    private double raxonPos;
    private double laxonPos;
    private double slowModeMultiplier = 0.5;
    private double angleToRot;


    private PathChain parkingSpace, scoringSpot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84, 36, Math.toRadians(0)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(IMU.class, "imu");
        gate = hardwareMap.get(Servo.class, "gate");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.REVERSE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood = hardwareMap.get(Servo.class, "hood");
        blocker = hardwareMap.get(Servo.class, "blocker");

        flywheelVelocity = 1800;
        intakeOn = false;
        flywheelOn = false;
        feederOn = false;
        kickerpos = true;
        debounceA = false;
        debounceB = false;
        debounceX = false;
        debounce_dpad_up = false;
        debounce_dpad_down = false;
        debounceY = false;
        debounceBACK = false;


        actiontimer = new Timer();

        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");


        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(23.687, 119.835))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();

        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(23.687, 119.835))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
        follower.setMaxPower(.8);
        blocker.setPosition(.3);
        raxon.setPosition(.5);
        laxon.setPosition(.5);
        hood.setPosition(.5694);
        imu.resetYaw();
        //Parallel: .4889
        //Min Values: .1894
        //Max Values: 1
        //R45 = .3389
        //B45 = .6094
        //AxonRot CCW = .2705/90

    }
    @Override
    public void loop() {




        if(autoTarget)
        {
            x = follower.getPose().getX();
            y = follower.getPose().getY();
            angleToRot = (imu.getRobotYawPitchRollAngles().getYaw()) - Math.toDegrees(Math.atan((138-y)/(138-x)));
            laxonPos = .4889 + (.2705/90)*angleToRot;
            raxonPos = .4889 + (.2705/90)*angleToRot;
        }




        //
        //raxonPos = (.5+(.43/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) + (.43/180)*(imu.getRobotYawPitchRollAngles().getYaw()));
        //laxonPos = (.5-(.57/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) - (.57/180)*(imu.getRobotYawPitchRollAngles().getYaw()));

        distance = Math.sqrt(Math.pow(144-y,2) + Math.pow(144-x,2));



        flywheelVelocity = .0701544 * Math.pow(distance,2) - 3.07502 * distance + 1626.87017;
        //hood.setPosition(.259228 * Math.sin(.03483 * distance + .48236) + .752718);
        if (gamepad1.left_stick_button){
            gate.setPosition(0);
        }
        if ( gamepad1.right_stick_button){
            gate.setPosition(.2);
        }

        if(raxonPos > 1)
        {
            raxonPos = 1;
        }
        if(raxonPos < 0)
       {
            raxonPos = 0;
        }
        if(laxonPos < 0)
        {
            laxonPos = 0;
        }
        if(laxonPos > 1)
        {
            laxonPos = 1;
        }

        raxon.setPosition(raxonPos);
        laxon.setPosition(laxonPos);

        if(!gamepad1.back){

            debounceBACK = true;
        }

        if (actiontimer.getElapsedTimeSeconds() > 4){
            kickerpos = false;
            blocker.setPosition(.3);



        }
//        if (gamepad1.back && debounceBACK && kickerpos){
//            kickerpos = false;
//            blocker.setPosition(.3);
//            actiontimer.resetTimer();
//            debounceBACK = false;
//
//
//
//        }
        if (gamepad1.back && debounceBACK && !kickerpos){
            blocker.setPosition(.50 );
            kickerpos = true;
            debounceBACK = false;
            actiontimer.resetTimer();
        }

        if (gamepad1.a && !intakeOn && !debounceA){
            debounceA = true;
            intakeOn = true;
            intakeOuter.setPower(-.8);
            intakeInner.setPower(.4);
        }
        if (gamepad1.a && intakeOn && !debounceA){
            debounceA = true;
            intakeOn = false;
            intakeOuter.setPower(0);
            intakeInner.setPower(0);
        }
        if (gamepad1.b && !feederOn && !debounceB){
            debounceB = true;
            feederOn = true;
            intakeOuter.setVelocity(900);
            intakeInner.setVelocity(-900);
//            feederL.setPower(-1);
//            feederR.setPower(1);
        }

        if (gamepad1.left_bumper && debounceLB){
            hood.setPosition(hood.getPosition()-.05);
            debounceLB = false;
        }
        if(!gamepad1.left_bumper)
        {
            debounceLB = true;
        }
        if (gamepad1.right_bumper && debounceRB){
            hood.setPosition(hood.getPosition()+.05);
            debounceRB = false;
        }
        if(!gamepad1.right_bumper)
        {
            debounceRB = true;
        }

        if (gamepad1.guide && debounceGUIDE){
            slowMode = !slowMode;
            debounceGUIDE = false;
        }
        if (!gamepad1.guide){
            debounceGUIDE = true;
        }





        if (gamepad1.b && feederOn && !debounceB){
            debounceB = true;
            feederOn = false;
            intakeOuter.setVelocity(0);
            intakeInner.setVelocity(0);
//            feederL.setPower(0);
//            feederR.setPower(0);
        }

        if (gamepad1.x && !flywheelOn && !debounceX){
            debounceX = true;
            flywheelOn = true;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        }
        if (gamepad1.x && flywheelOn && !debounceX){
            debounceX = true;
            flywheelOn = false;
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
        }

        if (gamepad1.left_trigger > .01 && debounceLEFT_TRIGGER){
            raxonPos = raxon.getPosition() +.02;
            laxonPos = laxon.getPosition() - .02;
            laxon.setPosition(laxonPos);
            raxon.setPosition(raxonPos);

            debounceLEFT_TRIGGER = false;
        }
        if (gamepad1.right_trigger > .01 && debounceRIGHT_TRIGGER){
            raxonPos = raxon.getPosition() - .02;
            laxonPos = laxon.getPosition() + .02;
            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
            debounceRIGHT_TRIGGER = false;

        }




        if (gamepad1.left_trigger < .01){
            debounceLEFT_TRIGGER = true;
        }
        if (gamepad1.right_trigger < .01){
            debounceRIGHT_TRIGGER = true;
        }




        if(!gamepad1.a){
            debounceA = false;
        }
        if(!gamepad1.b){
            debounceB = false;
        }
        if(!gamepad1.x){
            debounceX = false;
        }

        if(gamepad1.dpad_left && debounceDL)
        {
            laxonPos = laxon.getPosition() + .005;
            laxon.setPosition(laxonPos);
            debounceDL = false;
        }
        if(gamepad1.dpad_right && debounceDR)
        {
            laxonPos = laxon.getPosition() - .005;
            laxon.setPosition(laxonPos);
            debounceDR = false;
        }
        if(!gamepad1.dpad_left)
        {
            debounceDL = true;
        }
        if(!gamepad1.dpad_right)
        {
            debounceDR = true;
        }
        //Call this once per loop
        follower.update();
        telemetryM.update();


        if(gamepad1.dpad_up && flywheelOn && !debounce_dpad_up){
            flywheelVelocity += 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_up = true;
        }
        if(gamepad1.dpad_down && flywheelOn && !debounce_dpad_down){
            flywheelVelocity -= 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce_dpad_down = true;
        }

        if(!gamepad1.dpad_up){
            debounce_dpad_up = false;
        }
        if(!gamepad1.dpad_down){
            debounce_dpad_down = false;
        }
        if(gamepad1.y && debounceY)
        {
            autoTarget = !autoTarget;
            laxonPos = .3389;
            raxonPos = .3389;
            debounceY = false;
        }
        if(!gamepad1.y)
        {
            debounceY = true;
        }

        if(gamepad1.start && debounceStart){
          //code here!
            blocker.setPosition(.3);
            debounceStart = false;
            ballsPassed = 0;
            flywheelOn = true;
            boolean debounceSensor = true;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            actiontimer.resetTimer();
            double OgHoodPos = hood.getPosition();
            while (ballsPassed < 3 && actiontimer.getElapsedTimeSeconds() < 6){
                blocker.setPosition(5);
                if (actiontimer.getElapsedTimeSeconds() >= 4){
                    //loop through balls here
                    intakeOuter.setPower(-.8);
                    intakeInner.setPower(.4);

                    if (distanceSensor.getDistance(DistanceUnit.CM) < 10 && debounceSensor){
                        timerA.resetTimer();
                        while (timerA.getElapsedTimeSeconds() < .2){

                        }
                        hood.setPosition(hood.getPosition() + .05);
                        ballsPassed++;
                        debounceSensor = false;



                    }
                    else if (distanceSensor.getDistance(DistanceUnit.CM) > 10 ){
                        debounceSensor = true;
                    }


                }

            }
            blocker.setPosition(.3);
            intakeOuter.setPower(0);
            intakeInner.setPower(0);
            flywheelLeft.setVelocity(-1);
            flywheelRight.setVelocity(-1);
            hood.setPosition(OgHoodPos);


        }
        if(!gamepad1.start){
            debounceStart = true;
        }



        if (!automatedDrive) {


            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y , 
                    -gamepad1.left_stick_x ,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing

        //Stop automated following if the follower is done


        //Slow Mode


        //Optional way to change slow mode strength


        //Optional way to change slow mode strength



         telemetry.addData("blocker pos",blocker.getPosition());
        telemetry.addData("Hood position", hood.getPosition());
        telemetry.addData("raxon",raxon.getPosition());
        telemetry.addData("laxon",laxon.getPosition());
        telemetry.addData("atr", angleToRot);
        telemetry.addData("flywheel velocity",flywheelLeft.getVelocity());
        telemetry.addData("debounce y", debounceY);
        telemetry.addData("Angle(y/x)", Math.toDegrees(Math.atan((144-y)/(144-x))));
        telemetry.addData("Angle(x/y)", Math.toDegrees(Math.atan((144-x)/(144-y))));
        telemetry.addData("position", follower.getPose());
        /*telemetryM.debug("position", follower.getPose()); */
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetry.addData("YAW", imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("distance", distance);
        telemetry.addData("Distance Sensor", distanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("gate", gate.getPosition() );
        telemetry.addData("balls shot this burst" ,ballsPassed );

    }
}

