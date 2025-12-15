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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "NOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO", group = "Examples")

public class BlueTeleOp extends OpMode {
    private Follower follower;

    private boolean debounce1;
    private Timer pathTimer;

    private  Timer actiontimer;

    private Servo raxon;

    private Servo laxon;
    private Servo kicker;
    private Servo hood;

    private boolean debounce2;

    private boolean debounce3;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;

    private DcMotorEx flywheelLeft;

    private DcMotorEx flywheelRight;

    private DcMotorEx intakeLeft;
    private boolean intakeOn;
    private boolean flywheelOn;

    private boolean kickerpos;

    private boolean debounce4;
    private boolean debounce5;
    private boolean debounce6;

    private  boolean debounce7;

    private boolean debounce8;

    private boolean debounce9;
    private boolean debounce10;

    private double flywheelVelocity;
    private boolean feederOn;
    private DcMotorEx intakeRight;

    private CRServo feederL;

    private CRServo feederR;

    private double slowModeMultiplier = 0.5;

    private PathChain parkingSpace, scoringSpot;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 83, Math.toRadians(180)));
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheelLeft = hardwareMap.get(DcMotorEx.class, "flyL");
        flywheelRight = hardwareMap.get(DcMotorEx.class, "flyR");
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelLeft.setDirection(DcMotor.Direction.REVERSE);
        flywheelRight.setDirection(DcMotor.Direction.FORWARD);

        intakeLeft = hardwareMap.get(DcMotorEx.class, "intL");
        intakeRight = hardwareMap.get(DcMotorEx.class, "intR");
        intakeLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        intakeLeft.setDirection(DcMotor.Direction.REVERSE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        feederL = hardwareMap.get(CRServo.class, "feederL");
        feederR = hardwareMap.get(CRServo.class, "feederR");
        hood = hardwareMap.get(Servo.class, "hood");
        kicker = hardwareMap.get(Servo.class, "kicker");
        feederL.setDirection(DcMotorSimple.Direction.FORWARD);
        feederR.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelVelocity = 1800;
        intakeOn = false;
        flywheelOn = false;
        feederOn = false;
        kickerpos = true;
        debounce1 = false;
        debounce2 = false;
        debounce3 = false;
        debounce4 = false;
        debounce5 = false;
        debounce6 = false;
        debounce7 = false;

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
        kicker.setPosition(.95);
        raxon.setPosition(.3083);
        laxon.setPosition(.2394);

    }
    @Override
    public void loop() {


        if(!gamepad1.back){
            debounce7 = true;
        }

        if (gamepad1.back && debounce7 && kickerpos){
            kickerpos = false;
            kicker.setPosition(0.4);
            actiontimer.resetTimer();
            debounce7 = false;



        }
        if(actiontimer.getElapsedTime() > 300 && !kickerpos)
        {
            kicker.setPosition(.95);
            kickerpos = true;

        }
//        if (actiontimer.getElapsedTime() >= 500 && !kickerpos){
//            kicker.setPosition(.95);
//            kickerpos = true;
//        }
//
        if (!gamepad1.back){
            debounce7 = true;
        }
        if (gamepad1.a && !intakeOn && !debounce1){
            debounce1 = true;
            intakeOn = true;
            intakeLeft.setVelocity(-1000);
            intakeRight.setVelocity(-1000);
        }
        if (gamepad1.a && intakeOn && !debounce1){
            debounce1 = true;
            intakeOn = false;
            intakeLeft.setVelocity(0);
            intakeRight.setVelocity(0);
        }
        if (gamepad1.b && !feederOn && !debounce2){
            debounce2 = true;
            feederOn = true;
            intakeLeft.setVelocity(900);
            intakeRight.setVelocity(900);
//            feederL.setPower(-1);
//            feederR.setPower(1);
        }

        if (gamepad1.left_bumper){
        hood.setPosition(hood.getPosition()-.01);
        }
        if (gamepad1.right_bumper){
            hood.setPosition(hood.getPosition()+.01);
        }

        if (gamepad1.guide && debounce8){
            slowMode = !slowMode;
            debounce8 = false;
        }
        if (!gamepad1.guide){
            debounce8 = true;
        }



        if (gamepad1.b && feederOn && !debounce2){
            debounce2 = true;
            feederOn = false;
            intakeLeft.setVelocity(0);
            intakeRight.setVelocity(0);
//            feederL.setPower(0);
//            feederR.setPower(0);
        }

        if (gamepad1.x && !flywheelOn && !debounce3){
            debounce3 = true;
            flywheelOn = true;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            feederL.setPower(-1);
            feederR.setPower(1);
            actiontimer.resetTimer();


        }
        if (gamepad1.x && flywheelOn && !debounce3){
            debounce3 = true;
            flywheelOn = false;
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            feederL.setPower(0);
            feederR.setPower(0);
        }

        if (gamepad1.left_trigger > .01 && debounce9){
            raxon.setPosition(raxon.getPosition() -.03);
            debounce9 = false;
            laxon.setPosition(laxon.getPosition()  + .03);
        }
        if (gamepad1.right_trigger > .01 && debounce10){
            raxon.setPosition(raxon.getPosition() +.03);
            debounce10 = false;
            laxon.setPosition(laxon.getPosition()  - .03);
        }

        if (gamepad1.left_trigger < .01){
            debounce9 = true;
        }
        if (gamepad1.right_trigger < .01){
            debounce10 = true;
        }

        if(gamepad1.yWasPressed() && !debounce6)
        {
            feederL.setPower(-1);
            feederR.setPower(1);
            debounce6 = true;
        }

        if(gamepad1.yWasReleased() && debounce6)
        {
            debounce6 = false;
            feederL.setPower(0);
            feederR.setPower(0);
        }
        if(!gamepad1.a){
            debounce1 = false;
        }
        if(!gamepad1.b){
            debounce2 = false;
        }
        if(!gamepad1.x){
            debounce3 = false;
        }


        //Call this once per loop
        follower.update();
        telemetryM.update();


        if(gamepad1.dpad_up && flywheelOn && !debounce4){
            flywheelVelocity += 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce4 = true;
        }
        if(gamepad1.dpad_down && flywheelOn && !debounce5){
            flywheelVelocity -= 200;
            flywheelLeft.setVelocity(flywheelVelocity);
            flywheelRight.setVelocity(flywheelVelocity);
            debounce5 = true;
        }

        if(!gamepad1.dpad_up){
            debounce4 = false;
        }
        if(!gamepad1.dpad_down){
            debounce5 = false;
        }

//        if(!gamepad1.y)
//        {
//            debounce6 = false;
//        }
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
        if (automatedDrive && (gamepad1.startWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode


        //Optional way to change slow mode strength


        //Optional way to change slow mode strength

        if (gamepad1.startWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        telemetry.addData("kicker",kicker.getPosition());
        telemetry.addData("Hood position", hood.getPosition());
        telemetry.addData("raxon",raxon.getPosition());
        telemetry.addData("laxon",laxon.getPosition());
        telemetry.addData("flywheel velocity",flywheelLeft.getVelocity());
        telemetry.addData("debounce y", debounce6);
        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}
