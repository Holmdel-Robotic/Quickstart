package org.firstinspires.ftc.teamcode.pedroPathing;

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
@Autonomous(name = "AutonNearPrototype", group = "Examples")
public class AutonNearPrototype extends OpMode {

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
    private final Pose sidePose = new Pose(8, 8, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(56, 8, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the wall.
    private final Pose pickup1Pose = new Pose(24, 36, Math.toRadians(0)); // Highest (First Set) of Artifacts.
//    private final Pose pickup3Pose = new Pose(42, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup1CPose = new Pose(22,18, Math.toRadians(0));

    //private final Pose pickup1C2Pose = new Pose(48,98, Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(24, 60, Math.toRadians(0)); // Second Row of Artifacts from the Spike Mark.
    private final Pose pickup2CPose = new Pose(23,35, Math.toRadians(0));

    //private final Pose pickup2C2Pose = new Pose(50,97, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(24, 84, Math.toRadians(0));
    private final Pose pickup3CPose = new Pose(24,36, Math.toRadians(0));

    private final Pose scorePose2 = new Pose(60,84, Math.toRadians(0));

    private Path scorePreload;

    private PathChain grab1, score1, grab2, score2, grab3, score3,grab4, score4, end;

    private Servo kicker;





    enum State {
        START,
        PICKUP1,
        PICKUP2,
        PICKUP3,
        PICKUP4,
        PICKUP5,

        SCORING,
        END

    }

    State state = State.START;
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(scorePose, sidePose));
        scorePreload.setConstantHeadingInterpolation(180);

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
        score1 = follower.pathBuilder()
                .addPath(new BezierLine(sidePose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        grab2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup1CPose, pickup1Pose))
                .setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        score2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()

        
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .setVelocityConstraint(0.01)
//                .build();
        grab3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup2CPose, pickup2Pose))
                .setTangentHeadingInterpolation()
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
        score3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        grab4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, pickup3CPose, pickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        score4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose2))
                .setTangentHeadingInterpolation()
                .build();
        end = follower.pathBuilder()
                .addPath(new BezierLine(scorePose2, pickup3Pose))
                .setTangentHeadingInterpolation().build();
    }

    public void autonomousPathUpdate() {
        switch (state) {
            case START:
                flywheelLeft.setVelocity(1700);
                flywheelRight.setVelocity(1700);
                hood.setPosition(.57);

                follower.setMaxPower(0.9);
                follower.followPath(score1);
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

                            follower.followPath(grab2, true);
                            setPathState(State.PICKUP2);
                            count++;
                        } else if (count == 2) {
                            follower.followPath(grab3, true);
                            setPathState(State.PICKUP3);
                            count++;
                        } else if (count == 3) {
                            follower.followPath(grab4);
                            setPathState(State.PICKUP4);
                            count++;

                        }
                        else if(count == 4)
                        {
                            follower.followPath(end);
                            setPathState(State.END);
                        }
                        else if(count == 0)
                        {
                            follower.followPath(scorePreload);
                            setPathState(State.START);
                        }

                    }
                }


                break;


            case PICKUP1:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */

                    follower.followPath(score1,true);
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
                    follower.followPath(score2, true);
                    setPathState(State.SCORING);
                }
                break;
            case PICKUP3:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(.9);
                    follower.followPath(score3, true);
                    setPathState(State.SCORING);
                    //IntakeInner.setVelocity(300);
                    //IntakeOuter.setVelocity(-300);

                }
                break;
            case PICKUP4:
                if(!follower.isBusy()){

                    follower.followPath(score4);
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
        follower.setStartingPose(scorePose);



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
