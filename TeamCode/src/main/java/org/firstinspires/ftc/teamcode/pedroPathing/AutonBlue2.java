package org.firstinspires.ftc.teamcode.pedroPathing;
import  com.bylazar.configurables.annotations.Configurable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
/*
New Pathing for Auton Left
 */
@Autonomous(name = "AutonBlue2", group = "Examples")
public class AutonBlue2 extends OpMode {

    private Servo hood;

    private Servo raxon;

    private Servo laxon;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx IntakeInner;

    private DcMotorEx IntakeOuter;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int count;
    private final Pose startPose = new Pose(24, 120, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(36, 108, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the wall.
    private final Pose pickup1Pose = new Pose(20, 84, Math.toRadians(0)); // Highest (First Set) of Artifacts.
//    private final Pose pickup3Pose = new Pose(42, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CPose = new Pose(22,100, Math.toRadians(0));

    private final Pose pickup1C2Pose = new Pose(48,98, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(20, 60, Math.toRadians(0)); // Second Row of Artifacts from the Spike Mark.
    private final Pose pickup2CPose = new Pose(23,89, Math.toRadians(0));

    private final Pose leverPose = new Pose(14,70,Math.toRadians(0));

    private final Pose leverCPose = new Pose(28,71,Math.toRadians(0));

    private final Pose leverC2Pose = new Pose(52,90,Math.toRadians(0));
    //private final Pose pickup2C2Pose = new Pose(50,97, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(20, 36, Math.toRadians(0));
    private final Pose pickup3CPose = new Pose(23,76, Math.toRadians(0));

    private final Pose pickup3C2Pose = new Pose(49,94, Math.toRadians(0));

    private final Pose pickup4Pose = new Pose(10,9,Math.toRadians(0));

    private final Pose pickup4CPose = new Pose(36,6,Math.toRadians(0));

    private final Pose pickup4C2Pose = new Pose(67,77,Math.toRadians(0));

    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, grabPickup2, leverPush, scorePickup2, grabPickup3, grabPickup4, scorePickup4, scorePickup3 ;

    private Servo kicker;





    enum State {
        START,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        PICKUP4,
        PICKUP5,
        LEVER,

        SCORING,
        END

    }

    State state = State.START;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setTangentHeadingInterpolation();

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        raxon = hardwareMap.get(Servo.class,"raxon");
        laxon = hardwareMap.get(Servo.class,"laxon");

        IntakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        IntakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        kicker = hardwareMap.get(Servo.class, "blocker");
        IntakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeOuter.setDirection(DcMotor.Direction.REVERSE);
        IntakeInner.setDirection(DcMotor.Direction.FORWARD);
        IntakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        hood = hardwareMap.get(Servo.class, "hood");


        raxon.setPosition(.3389);
        laxon.setPosition(.3389);
        kicker.setPosition(.4);
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); *


    //Parallel: .4889
    //Min Values: .1894
    //Max Values: 1
    //R45 = .3389
    //B45 = .6094
    //AxonRot = .2705/90
     */
        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1CPose, pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup1Pose, pickup1C2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()

        
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .setVelocityConstraint(0.01)
//                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2CPose, pickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        leverPush = follower.pathBuilder()
                .addPath(new BezierCurve(pickup2Pose, leverCPose, leverPose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        scorePickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(pickup2Pose, pickup3Pose))
//                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), pickup3Pose.getHeading())
//                .setVelocityConstraint(.01)
//                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        /*grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(leverPose ,leverC2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3CPose, pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup3Pose, pickup3C2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup4CPose, pickup4Pose))
                .setTangentHeadingInterpolation()
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(pickup4Pose, pickup4C2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (state) {
            case START:
                flywheelLeft.setVelocity(1700);
                flywheelRight.setVelocity(1700);
                hood.setPosition(.57);

                follower.setMaxPower(0.9);
                follower.followPath(scorePreload);
                setPathState(State.SCORING);
                actionTimer.resetTimer();


                break;
            //After First 3
            case SCORING:
                if(follower.isBusy())
                {
                    actionTimer.resetTimer();
                }
                else if (!follower.isBusy())
                {

                    if(actionTimer.getElapsedTimeSeconds() <= 1){

                        flywheelLeft.setVelocity(-1800);
                        flywheelRight.setVelocity(-1800);

                        hood.setPosition(.57);
                        // && flywheelRight.getVelocity() > 1650 && flywheelLeft.getVelocity() > 1650 &&
                    } else if (actionTimer.getElapsedTimeSeconds() >= 1 && actionTimer.getElapsedTimeSeconds() <= 3) {

                        IntakeOuter.setPower(-.8);
                        IntakeInner.setPower(-.4);
                        kicker.setPosition(.5);


                    }
                    else if (actionTimer.getElapsedTimeSeconds() >= 3 && actionTimer.getElapsedTimeSeconds() <= 3.1) {
                        kicker.setPosition(.3);


                    }
                    else {

                        //IntakeInner.setVelocity(0);
                        //IntakeOuter.setVelocity(0);
                        flywheelLeft.setVelocity(-.01);
                        flywheelRight.setVelocity(-.01);
                        if (count == 1) {

                            follower.followPath(grabPickup1, true);
                            setPathState(State.PICKUP1);
                            count++;
                        } else if (count == 2) {
                            follower.followPath(grabPickup2, true);
                            setPathState(State.PICKUP2);
                            count++;
                        } else if (count == 3) {
                            follower.followPath(grabPickup3);
                            setPathState(State.PICKUP3);
                            count++;

                        }
                        else if(count == 4)
                        {
                            follower.followPath(grabPickup4);
                            setPathState(State.PICKUP4);
                            count++;
                        }
                        else if(count == 5)
                        {
                            follower.followPath(grabPickup1);
                            setPathState(State.END);
                        }

                    }
                }


                break;


            case PICKUP1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(scorePickup1,true);
                    setPathState(State.SCORING);
                    //IntakeInner.setVelocity(300);
                    //IntakeOuter.setVelocity(-300);

                }
                break;
//            case PICKUP2:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
//                if(!follower.isBusy()) {
//                    /* Grab Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
//                    follower.followPath(scorePickup2,true);
//                    setPathState(State.PICKUP2RETURN);
//                }
//                break;
//            case 5:
//                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
//                if(!follower.isBusy()) {
//                    /* Score Sample */
//
//                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
//                    follower.followPath(grabPickup3,true);
//                    setPathState(6);
//                }
//                break;

            case PICKUP2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(.75);
                    follower.followPath(leverPush, true);
                    setPathState(State.LEVER);
                }
                break;
            case LEVER:
                if(!follower.isBusy())
                {
                    follower.followPath(leverPush, true);
                    setPathState(State.SCORING);
                }
            case PICKUP3:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(.9);
                    follower.followPath(scorePickup3, true);
                    setPathState(State.SCORING);
                    //IntakeInner.setVelocity(300);
                    //IntakeOuter.setVelocity(-300);

                }
                break;
            case PICKUP4:
                if(!follower.isBusy()){

                    follower.followPath(scorePickup4);
                    setPathState(State.SCORING);
                }
                break;
//            case PICKUP5:
//                if(!follower.isBusy()){
//
//                    follower.followPath(scorePickup3);
//                    setPathState(State.SCORING);
//                }
//                break;


            case END:
                if(!follower.isBusy()){


                }
                break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(State stateCooler) {
        state = stateCooler;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", state);
        telemetry.addData("Shooter velocity L", flywheelLeft.getVelocity());
        telemetry.addData("Shooter velocity R", flywheelRight.getVelocity());
        telemetry.addData("Actiontimer",actionTimer.getElapsedTimeSeconds());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("count", count);
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer.resetTimer();
        count = 1;

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);



    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(State.START);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }
}
