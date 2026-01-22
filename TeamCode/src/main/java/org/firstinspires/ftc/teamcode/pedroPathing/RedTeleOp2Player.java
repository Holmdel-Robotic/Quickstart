package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class RedTeleOp2Player extends OpMode {
    private Follower follower;

    private boolean readyToRotate;
    private boolean debounceA;
    private Timer pathTimer;

    private  Timer actiontimer;

    private  Timer aTimer;

    private Servo raxon;

    private  Servo gate;
    private Servo laxon;
    private Servo blocker;
    private Servo hood;

    private double x;
    private double y;

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
    private boolean debounceRIGHT_TRIGGER;
    private boolean debounceLEFT_TRIGGER;
    private boolean debounceGUIDE;

    private boolean dLT1, dLT2;
    private boolean dRT1, dRT2;

    private boolean dLB1, dRB1, dLB2, dRB2;
    private boolean dA, dB, dX, dY, dG;

    private boolean dUp, dDown;


    private double flywheelVelocity, intakeVelocity;
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
        dA = false;
        dB = false;
        dX = false;
        dUp = false;
        dDown = false;
        dY = false;
        dG = false;
        dLB1 = false;
        dRB2 = false;
        dLB2 = false;
        dRB1 = false;

        aTimer = new Timer();
        actiontimer = new Timer();

        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");
        laxon.setDirection(Servo.Direction.FORWARD);
        raxon.setDirection(Servo.Direction.FORWARD);


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
        //Parallel: .5
        //Min Values: .1
        //Max Values: 1
        //R45 = .36
        //B45 = .64
        //AxonRot CCW = .28/90

    }
    @Override
    public void loop() {






        if(autoTarget && aTimer.getElapsedTime() > 500)
        {


            x = follower.getPose().getX();
            y = follower.getPose().getY();
            angleToRot = (imu.getRobotYawPitchRollAngles().getYaw()) - Math.toDegrees(Math.atan((138-y)/(138-x)));
            laxonPos = .5 + (.28/90)*angleToRot;
            raxonPos = .5 + (.28/90)*angleToRot;
            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
            aTimer.resetTimer();
        }

        /* hypothetical hood stuff
        hoodPos = Math.atan(38.75/distance) * .01684
         */


        //
        //raxonPos = (.5+(.43/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) + (.43/180)*(imu.getRobotYawPitchRollAngles().getYaw()));
        //laxonPos = (.5-(.57/180)*(Math.toDegrees(Math.atan((144-y)/(144-x)))) - (.57/180)*(imu.getRobotYawPitchRollAngles().getYaw()));

        distance = Math.sqrt(Math.pow(144-y,2) + Math.pow(144-x,2));
        //change 144 to 138



        //flywheelVelocity = .0701544 * Math.pow(distance,2) - 3.07502 * distance + 1626.87017;
        //hood.setPosition(.259228 * Math.sin(.03483 * distance + .48236) + .752718);
        if (gamepad1.a && !dA){
            if(gate.getPosition() != 0)
            {
                gate.setPosition(0);
            }
            else
            {
                gate.setPosition(.2);
            }
            dA = false;
        }

        if(raxonPos > 1)
        {
            raxonPos = 1;
        }
        if(raxonPos < .1)
       {
            raxonPos = .1;
        }
        if(laxonPos < .1)
        {
            laxonPos = .1;
        }
        if(laxonPos > 1)
        {
            laxonPos = 1;
        }



        if(!gamepad2.b){

            dB = true;
        }

        if (actiontimer.getElapsedTimeSeconds() > 4){
            kickerpos = false;
            blocker.setPosition(.3);



        }
//        if (gamepad2.b && dB && kickerpos){
//            kickerpos = false;
//            blocker.setPosition(.3);
//            actiontimer.resetTimer();
//            debounceBACK = false;
//
//
//
//        }
        if (gamepad2.b && dB && !kickerpos){
            blocker.setPosition(.50 );
            kickerpos = true;
            dB = false;
            actiontimer.resetTimer();
        }

        if (gamepad1.x && !intakeOn && !dX){
            dX = true;
            intakeOn = true;
            intakeOuter.setVelocity(intakeVelocity);
            intakeInner.setVelocity(-intakeVelocity);
        }
        if(gamepad1.right_trigger > .01&& !dRT1)
        {
            dRT1 = false;
            intakeVelocity += 50;
            intakeOuter.setVelocity(intakeVelocity);
            intakeInner.setVelocity(-intakeVelocity);
        }
        if(gamepad1.left_trigger > .01 && !dLT1)
        {
            dLT1 = false;
            intakeVelocity -= 50;
            intakeOuter.setVelocity(intakeVelocity);
            intakeInner.setVelocity(-intakeVelocity);
        }
        if(gamepad1.right_bumper && !dRB1)
        {
            intakeVelocity *= -1;
            dRB1 = false;
        }
        if(gamepad1.left_trigger < .01)
        {
            dLT1 = true;
        }
        if(gamepad1.right_trigger < .01)
        {
            dRT1 = true;
        }
        if(!gamepad1.right_bumper)
        {
            dRB1 = true;
        }
        if (gamepad1.x && intakeOn && !dX){
            dX = true;
            intakeOn = false;
            intakeOuter.setPower(0);
            intakeInner.setPower(0);
        }
//        if (gamepad1.b && !feederOn && !debounceB){
//            debounceB = true;
//            feederOn = true;
//            intakeOuter.setVelocity(900);
//            intakeInner.setVelocity(-900);
////            feederL.setPower(-1);
////            feederR.setPower(1);
//        }

        if (gamepad2.left_bumper && !dLB2){
            hood.setPosition(hood.getPosition()-.05);
            dLB2 = true;
        }
        if(!gamepad2.left_bumper)
        {
            dLB2 = false;
        }
        if (gamepad2.right_bumper && !dRB2){
            hood.setPosition(hood.getPosition()+.05);
            dRB2 = true;
        }
        if(!gamepad2.right_bumper)
        {
            dRB2 = false;
        }

        if (gamepad1.guide && debounceGUIDE){
            slowMode = !slowMode;
            debounceGUIDE = false;
        }
        if (!gamepad1.guide){
            debounceGUIDE = true;
        }





//        if (gamepad1.b && feederOn && !debounceB){
//            debounceB = true;
//            feederOn = false;
//            intakeOuter.setVelocity(0);
//            intakeInner.setVelocity(0);
////            feederL.setPower(0);
////            feederR.setPower(0);
//        }

        if (gamepad2.y && !flywheelOn && !dY){
            dY = true;
            flywheelOn = true;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
        }
        if (gamepad2.y && flywheelOn && !dY){
            dY = true;
            flywheelOn = false;
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
        }

        if (gamepad2.left_trigger > .01 && !dLT2){
            raxonPos = raxon.getPosition() +.03;
            laxonPos = laxon.getPosition() + .03;
            laxon.setPosition(laxonPos);
            raxon.setPosition(raxonPos);

            dLT2 = true;
        }
        if (gamepad2.right_trigger > .01 && !dRT2){
            raxonPos = raxon.getPosition() - .03;
            laxonPos = laxon.getPosition() - .03;
            raxon.setPosition(raxonPos);
            laxon.setPosition(laxonPos);
            dRT2 = true;

        }




        if (gamepad2.left_trigger < .01){
            dLT2 = false;
        }
        if (gamepad2.right_trigger < .01){
            dRT2 = false;
        }




        if(!gamepad1.a){
            dA = false;
        }
        if(!gamepad2.b){
            dB = false;
        }
        if(!gamepad1.x){
            dX = false;
        }

        if(gamepad1.dpad_left && !debounceDL)
        {
            laxonPos = laxon.getPosition() + .005;
            laxon.setPosition(laxonPos);
            debounceDL = true;
        }
        if(gamepad1.dpad_right && !debounceDR)
        {
            laxonPos = laxon.getPosition() - .005;
            laxon.setPosition(laxonPos);
            debounceDR = true;
        }
        if(!gamepad1.dpad_left)
        {
            debounceDL = false;
        }
        if(!gamepad1.dpad_right)
        {
            debounceDR = false;
        }
        //Call this once per loop
        follower.update();
        telemetryM.update();


        if(gamepad2.dpad_up && flywheelOn && !dUp){
            flywheelVelocity += 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            dUp = true;
        }
        if(gamepad2.dpad_down && flywheelOn && !dDown){
            flywheelVelocity -= 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            dDown = true;
        }

        if(!gamepad2.dpad_up){
            dUp = false;
        }
        if(!gamepad2.dpad_down){
            dDown = false;
        }
        if(gamepad2.guide && !dG)
        {
            autoTarget = !autoTarget;
            laxonPos = .5;
            raxonPos = .5;
            dG = true;
        }
        if(!gamepad2.y)
        {
            dY = false;
        }
        if(!gamepad2.guide)
        {
            dG = false;
        }

        if(gamepad2.start && debounceStart){
          //code here!
            blocker.setPosition(.3);
            debounceStart = false;
            double ballsPassed = 0;
            flywheelOn = true;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            actiontimer.resetTimer();
            double OgHoodPos = hood.getPosition();
            while (ballsPassed < 3 && actiontimer.getElapsedTimeSeconds() < 4){
                blocker.setPosition(5);
                if (actiontimer.getElapsedTimeSeconds() >= 2){
                    //loop through balls here
                    intakeOuter.setPower(-.8);
                    intakeInner.setPower(.4);

                    if (distanceSensor.getDistance(DistanceUnit.CM) < 10){
                        hood.setPosition(hood.getPosition() + .02);
                        ballsPassed++;
                    }


                }

            }
            blocker.setPosition(.3);
            intakeOuter.setPower(0);
            intakeInner.setPower(0);


        }
        if(!gamepad2.start){
            debounceStart = true;
        }



        if (!automatedDrive) {


            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
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

    }
}