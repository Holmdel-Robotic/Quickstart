package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
//
@Autonomous(name = "Blue  Small Triangle", group = "Autonomous")
@Config
public class BlueSmallTriangle extends OpMode {

    private Follower follower;
    private DcMotorEx frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, intakeOuter, intakeInner, flywheelLeft, flywheelRight;

    private Servo hood, raxon, laxon, innerGate, outerGate;

    public static double VELO = 2200, SERVOPOS = .285, HOODPOS = .33, ADJUSTEDSERVOPOS = .55, loops = 0;

    private TelemetryManager panelsTelemetry;
    private Timer timer2;
    private ElapsedTime pathTimer;
    private Timer actionTimer;
    private Paths paths;

//    private boolean scored = false
    private boolean firstTime = true;
    private boolean pickedUp = false;

    private Servo indicatorLight;
    private double GREEN = .5;
    private double BLUE = .6;


    private enum State {
        START,

        FLYWHEELRAMPUP,
        SCORE,
        PICKUP1,
        HUMANZONE,
        END;
    }

    private State pathState = State.START;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        indicatorLight = hardwareMap.get(Servo.class, "light");
        //indicatorLight2 = hardwareMap.get(Servo.class, "lightTwo");

        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "fl");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "fr");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "bl");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "br");
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intInner");
        intakeOuter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeInner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeOuter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeInner.setDirection(DcMotor.Direction.REVERSE);
        intakeOuter.setDirection(DcMotor.Direction.FORWARD);

        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.FORWARD);
        flywheelRight.setDirection(DcMotor.Direction.REVERSE);

        hood = hardwareMap.get(Servo.class, "hood");
        raxon = hardwareMap.get(Servo.class, "raxon");
        laxon = hardwareMap.get(Servo.class, "laxon");
        raxon.setPosition(ADJUSTEDSERVOPOS);
        laxon.setPosition(ADJUSTEDSERVOPOS);
        innerGate = hardwareMap.get(Servo.class, "innerGate");
        outerGate = hardwareMap.get(Servo.class, "outerGate");
        outerGate.setPosition(0);
        hood.setDirection(Servo.Direction.REVERSE);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.800, 9, Math.toRadians(90)));
        timer2 = new Timer();
        pathTimer = new ElapsedTime();
        actionTimer = new Timer();
        paths = new Paths(follower);
        pathState = State.START;
        innerGate.setPosition(.2);
        firstTime = true;
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        telemetry.addData("flywheelV", flywheelLeft.getVelocity());
        telemetry.addData("FirstTime", firstTime);
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("actionTimer", actionTimer.getElapsedTimeSeconds());
        panelsTelemetry.debug("isBusy?", follower.isBusy());
        panelsTelemetry.debug("pathTime", pathTimer.seconds());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(47.800, 13.000),
                                    new Pose(22.203, 23.234),
                                    new Pose(24.000, 33.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(100))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(24.000, 33.000),
                                    new Pose(22.609, 23.031),
                                    new Pose(47.800, 13.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(100), Math.toRadians(180))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.800, 13.000),
                                    new Pose(6.700, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(6.700, 13.000),
                                    new Pose(47.800, 13.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(47.800, 13.000),
                                    new Pose(60, 36))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }
    public static Pose mirror(Pose p) {
        return new Pose(144 - p.getX(), p.getY(), Math.toRadians(180 - Math.toDegrees(p.getHeading())));
    }
    public void autonomousPathUpdate() {
        switch (pathState) {
            case START:
                innerGate.setPosition(.2);
                raxon.setPosition(ADJUSTEDSERVOPOS);
                laxon.setPosition(ADJUSTEDSERVOPOS);
                flywheelLeft.setVelocity(VELO);
                flywheelRight.setVelocity(VELO);
                intakeOuter.setPower(.9);
                setPathState(State.FLYWHEELRAMPUP);
                actionTimer.resetTimer();
                timer2.resetTimer();
                outerGate.setPosition(0);
                indicatorLight.setPosition(BLUE);
                break;

            case FLYWHEELRAMPUP:
                firstTime = true;
                if (actionTimer.getElapsedTimeSeconds() > 3){
                    actionTimer.resetTimer();
                    setPathState(State.SCORE);
                    innerGate.setPosition(.575);
                    outerGate.setPosition(0);
                    firstTime = true;
                }
                break;

            case SCORE:
                if (timer2.getElapsedTimeSeconds() > 1){
                    outerGate.setPosition(.6);
                    intakeInner.setPower(.9);
                    indicatorLight.setPosition(GREEN);

                }

                if (follower.isBusy() || firstTime) {
                    actionTimer.resetTimer();
                    firstTime = false;
                }

                else {

                    if (!pickedUp){
                        raxon.setPosition(ADJUSTEDSERVOPOS);
                        laxon.setPosition(ADJUSTEDSERVOPOS);
                    }else{
                        raxon.setPosition(SERVOPOS);
                        laxon.setPosition(SERVOPOS);
                    }


                    flywheelLeft.setVelocity(VELO);
                    flywheelRight.setVelocity(VELO);
                    hood.setPosition(HOODPOS);
                    innerGate.setPosition(.575);
                    outerGate.setPosition(.6);
                    intakeInner.setPower(.9);
                    indicatorLight.setPosition(GREEN);
                    if (actionTimer.getElapsedTimeSeconds() > 2) {

                        indicatorLight.setPosition(BLUE);
                        intakeInner.setPower(0);
                        innerGate.setPosition(.2);
                        outerGate.setPosition(0);
                        if (!pickedUp) {
                            follower.followPath(paths.line1, true);
                            setPathState(State.PICKUP1);

                        }
                        else if (loops >= 3) {
                            follower.followPath(paths.line5);
                            setPathState(State.END);
                        }
                        else {
                            loops += 1;
                            follower.followPath(paths.line3, true);
                            setPathState(State.HUMANZONE);
                        }
                    }
                }
                break;

            case PICKUP1:
                if (!follower.isBusy()) {
                    pickedUp = true;
                    follower.followPath(paths.line2, true);
                    setPathState(State.SCORE);
                }
                break;

            case HUMANZONE:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line4, true);
                    setPathState(State.SCORE);
                    timer2.resetTimer();
                }
                break;

            case END:
                if (!follower.isBusy()) {
                    requestOpModeStop();
                }
                break;
        }
    }
    public void setPathState(State pState) {
        pathState = pState;
        pathTimer.reset();
    }
}
