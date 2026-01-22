package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
 // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "AutonRed", group = "Examples")
public class AutonRealRight extends OpMode {

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
    private final Pose startPose = new Pose(144-23.687, 119.835, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(144-58, 85, Math.toRadians(0)); // Scoring Pose of our robot. It is facing the wall.
    private final Pose pickup1Pose = new Pose(144-20, 83, Math.toRadians(0)); // Highest (First Set) of Artifacts.
//    private final Pose pickup3Pose = new Pose(42, 60, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.

    private final Pose pickup2Pose = new Pose(144-20, 60, Math.toRadians(0)); // Second Row of Artifacts from the Spike Mark.

    private final Pose pickup3CPose = new Pose(144-60, 36, Math.toRadians(0));

    private final Pose pickup3Pose = new Pose(144-20, 36, Math.toRadians(0));
    private final Pose pickup2CPose = new Pose(144-60,60,Math.toRadians(0)); // Adithiya's Bezier curve control point yay
    private Path scorePreload;

    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, grabPickup4, grabPickup5, scorePickup3 ;

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
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

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


        raxon.setPosition(.36);
        laxon.setPosition(.36);
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
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .setVelocityConstraint(0.01)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
//        grabPickup2 = follower.pathBuilder()
//                .addPath(new BezierLine(scorePose, pickup2Pose))
//                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
//                .setVelocityConstraint(0.01)
//                .build();
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2CPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2CPose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2CPose, pickup2Pose))
                .setLinearHeadingInterpolation(pickup2CPose.getHeading(), pickup2Pose.getHeading())
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
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .setVelocityConstraint(.01)
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3CPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3CPose.getHeading())
                .build();

        grabPickup5 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3CPose, pickup3Pose))
                .setLinearHeadingInterpolation(pickup3CPose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .setVelocityConstraint(.01)
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
                            follower.followPath(grabPickup4);
                            setPathState(State.PICKUP4);
                            count++;

                        }
                        else if(count == 4)
                        {
                            follower.followPath(grabPickup4);
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
                    follower.followPath(grabPickup3, true);
                    setPathState(State.PICKUP3);
                }
                break;
            case PICKUP3:
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.setMaxPower(.9);
                    follower.followPath(scorePickup2, true);
                    setPathState(State.SCORING);
                    //IntakeInner.setVelocity(300);
                    //IntakeOuter.setVelocity(-300);

                }
                break;
            case PICKUP4:
                if(!follower.isBusy()){

                    follower.followPath(grabPickup5);
                    setPathState(State.PICKUP5);
                }
                break;
            case PICKUP5:
                if(!follower.isBusy()){

                    follower.followPath(scorePickup3);
                    setPathState(State.SCORING);
                }
                break;


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
