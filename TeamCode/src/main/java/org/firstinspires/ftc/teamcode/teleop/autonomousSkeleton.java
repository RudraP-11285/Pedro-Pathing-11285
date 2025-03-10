package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Example Auto Blue", group = "TeleOp")
public class autonomousSkeleton extends OpMode {
    //region Declare Hardware
    // Declare OpMode members for each of the 4 drive motors and 3 horizontal/vertical lift motors
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null; // Spongebob
    private DcMotor leftBackDrive = null; // Clarence
    private DcMotor rightFrontDrive = null; // Dumbledore
    private DcMotor rightBackDrive = null; // Gandalf
    private DcMotor verticalRight = null; // Aristotle
    private DcMotor verticalLeft = null; // Plato
    private DcMotor horizontalDrive = null; // Pythagoras

    // All 5 of the Intake Servos plugged into Expansion Hub 3
    private Servo intakeArm =  null; // Edward
    private Servo intakeClaw =  null; // Servo that opens and closes intake claw
    private Servo intakeRotate =  null; // Servo that rotates the claw left right
    private Servo intakeWrist =  null; // Servo that rotates the claw up down

    // All 3 of the Outtake Servos plugged into Control Hub
    private Servo deposClaw =  null; // Edward
    private Servo deposLeft =  null; // Stuart
    private Servo deposRight =  null; // Felicia

    private AnalogInput depositEncoder1 = null;
    private AnalogInput depositEncoder2 = null;

    private AnalogInput wristEncoder1 = null;
    private AnalogInput wristEncoder2 = null;
    private AnalogInput armEncoder1 = null;
    private AnalogInput armEncoder2 = null;

    private DigitalChannel magLimVertical1 = null;
    private DigitalChannel magLimVertical2 = null;
    private DigitalChannel magLimHorizontal1 = null;
    private DigitalChannel magLimHorizontal2 = null;

    ContinuousServoController deposLeftController = null;
    ContinuousServoController deposRightController = null;
    ContinuousServoController wristServoController = null;
    ContinuousServoController intakeArmServoController = null;

    CRServo dummy = null;

    // Sensors
    private DistanceSensor backDistance = null;
    private Limelight3A limelight;

    boolean magHorOn = true; // Usually, "false" means pressed
    boolean magVertOn = true; // Usually, "false" means pressed
    boolean limitSwitchNotTriggered = true;
    //endregion

    //region Set States
    String robotState = "Transfer";
    String scoreState = "Sample";
    //endregion

    //region Declare Booleans
    // Servo Toggle Debounces
    Boolean intakeClawState = false; // true = open, false = close (i think)
    Boolean intakeRotateState = false; // false = transfer rotation
    Boolean deposClawState = true; // true = open, false = close
    Boolean deposArmState = false;

    Boolean intakeClawDebounce = false; // Claw Open
    Boolean intakeRotateDebounce = false; // Rotated in State 1
    Boolean deposClawDebounce = false;
    Boolean deposArmDebounce = false;

    Boolean horizontalDriveLockDebounce = false;
    Boolean horizontalDriveLockState = false;

    Boolean intakeState = true;
    Boolean intakeDebounce = false;

    Boolean grabbing = false;
    Boolean intakeWaitToReturn = false;
    double grabTimer = 0.0;

    Boolean autoIntakeMode = false;
    Boolean autoIntakeDebounce = false;
    //endregion

    private double timeStamp = 0.0;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /** Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    //region Declare Poses
    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 96, Math.toRadians(0));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(16.25, 133.5, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(32.5, 123.75, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(32.5, 133.2, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(43, 128.79, Math.toRadians(90));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(64.25, 98, Math.toRadians(90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(64.25091633466136, 129.0836653386454, Math.toRadians(90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path scorePreload, park;
    private PathChain grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;
    //endregion

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scorePose)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierCurve(new Point(scorePose), /* Control Point */ new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Move to Score, Bring Lift Up
                deposClawState = true; // Close Depos Claw
                intakeClawState = false; // Open Intake Claw
                intakeRotateState = false; // Rotate Intake (Transfer Position)
                follower.followPath(scorePreload, true); // Move to score position

                if (verticalRight.getCurrentPosition() < 3150) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                }

                if (follower.getPose().getX() > (scorePose.getX() - 0.75) && follower.getPose().getY() > (scorePose.getY() - 0.75) && (!(verticalRight.getCurrentPosition() < 3150))) {
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    setPathState(1);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 1: // Bring Up Arm and Drop
                deposArmState = true;
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(2);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 2: // Bring the Depos Arm Back Down
                deposArmState = true;
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(3);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 3: // Bring the arm back, lift down, and move to grab
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) {
                    break;
                }
                deposArmState = false;
                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(0.5);
                    verticalRight.setPower(-0.5);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }
                follower.followPath(grabPickup1, false);

                if(follower.getPose().getX() > (pickup1Pose.getX() - 0.75) && follower.getPose().getY() > (pickup1Pose.getY() - 0.75)) {
                    /* Score Preload */
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(scorePreload,true);
                    setPathState(4);
                }
                break;
            case 4: // Keep bringing arm down, extend forward and pick up
                LLResult cameraResult = limelight.getLatestResult();
                double[] pythonOutputs = cameraResult.getPythonOutput();

                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }


                if (horizontalDrive.getCurrentPosition() < 75) {
                    horizontalDrive.setPower(0.2);
                } else {
                    horizontalDrive.setPower(0);
                }


                double lateral = 0;
                if (pythonOutputs[6] > 25) {
                    lateral = 0.3;
                } else if (pythonOutputs[6] < -25) {
                    lateral = -0.3;
                } else {
                    lateral = 0;
                }
                double leftFrontPower  =  lateral;
                double rightFrontPower = -lateral;
                double leftBackPower   = -lateral;
                double rightBackPower  =  lateral;
                leftFrontDrive.setPower(leftFrontPower);
                rightFrontDrive.setPower(rightFrontPower);
                leftBackDrive.setPower(leftBackPower);
                rightBackDrive.setPower(rightBackPower);


                // Rotate Wrist and Arm Servos into POSITION!
                intakeState = false;
                if (pythonOutputs[5] > 0.5 && lateral == 0 && (Math.abs(intakeArmServoController.getCurrentPositionInDegrees() - 51.5) < 2)) {
                    grabbing = true;
                    grabTimer = runtime.seconds();
                    setPathState(5);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 5: // Hold the arm for a tiny bit to finish grabbing
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(6);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 6: // Move to score, extend back and get ready
                if (!magHorOn) {
                    horizontalDrive.setPower(-0.2);
                } else {
                    horizontalDrive.setPower(0);
                }

                intakeState = true;
                deposArmState = false;

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.75)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(7);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 7: // Transfer the piece
                deposClawState = true; // Close Depos Claw
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.4)) {
                    break;
                }

                follower.followPath(scorePickup1, true);

                intakeClawState = false;
                setPathState(8);
                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                break;
            case 8: // Start lifting up once piece is transferred
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.20)) {
                    break;
                }

                if (verticalRight.getCurrentPosition() < 3150) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                } else {
                    setPathState(9);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 9: // Bring Depos Arm Up and Drop
                deposArmState = true;

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(10);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 10: // Bring the arm back, lift down, and move to grab
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) {
                    break;
                }
                deposArmState = false;

                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(0.5);
                    verticalRight.setPower(-0.5);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }
                follower.followPath(grabPickup2, true);

                if(follower.getPose().getX() > (pickup2Pose.getX() - 0.75) && follower.getPose().getY() > (pickup2Pose.getY() - 0.75)) {
                    /* Score Preload */
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(scorePreload,true);
                    setPathState(11);
                }
                break;
            case 11: // Keep bringing arm down, extend forward and pick up
                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                if (horizontalDrive.getCurrentPosition() < 125) {
                    horizontalDrive.setPower(0.2);
                } else {
                    horizontalDrive.setPower(0);
                }
                // Rotate Wrist and Arm Servos into POSITION!
                intakeState = false;
                if (Math.abs(intakeArmServoController.getCurrentPositionInDegrees() - 51.5) < 2) {
                    grabbing = true;
                    grabTimer = runtime.seconds();
                    setPathState(12);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 12: // Hold the arm for a tiny bit to finish grabbing
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(13);
                }
                break;
            case 13: // Move to score, extend back and get ready
                if (!magHorOn) {
                    horizontalDrive.setPower(-0.2);
                } else {
                    horizontalDrive.setPower(0);
                }

                intakeState = true;
                deposArmState = false;

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.75)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(14);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 14: // Transfer the piece
                deposClawState = true;
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.4)) {
                    break;
                }

                follower.followPath(scorePickup2, true);

                intakeClawState = false;
                setPathState(15);
                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                break;
            case 15: // Start lifting up once piece is transferred
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.25)) {
                    break;
                }

                if (verticalRight.getCurrentPosition() < 3150) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                } else {
                    setPathState(16);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 16: // Bring Depos Arm Up and Drop
                deposArmState = true;

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClawState = false;
                    setPathState(24);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 17: // Bring the arm back, lift down, and move to grab
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) {
                    break;
                }
                deposArmState = false;
                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(0.7);
                    verticalRight.setPower(-0.7);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }
                follower.followPath(grabPickup3, true);

                if(follower.getPose().getX() > (pickup3Pose.getX() - 0.75) && follower.getPose().getY() > (pickup3Pose.getY() - 0.75)) {
                    /* Score Preload */
                    verticalLeft.setPower(0);
                    verticalRight.setPower(0);
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(scorePreload,true);
                    setPathState(18);
                }
                break;
            case 18: // Keep bringing arm down, extend forward and pick up
                if (verticalRight.getCurrentPosition() > 5) {
                    verticalLeft.setPower(1);
                    verticalRight.setPower(-1);
                } else {
                    verticalRight.setPower(0);
                    verticalLeft.setPower(0);
                }

                if (horizontalDrive.getCurrentPosition() < 125) {
                    horizontalDrive.setPower(0.2);
                } else {
                    horizontalDrive.setPower(0);
                }
                intakeRotateState = true;
                // Rotate Wrist and Arm Servos into POSITION!
                intakeState = false;
                if (Math.abs(intakeArmServoController.getCurrentPositionInDegrees() - 51.5) < 2) {
                    grabbing = true;
                    grabTimer = runtime.seconds();
                    setPathState(19);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 19: // Hold the arm for a tiny bit to finish grabbing
                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    setPathState(20);
                }
                break;
            case 20: // Move to score, extend back and get ready
                follower.followPath(scorePickup3, true);
                intakeRotate.setPosition(1);

                if (!magHorOn) {
                    horizontalDrive.setPower(-0.2);
                } else {
                    horizontalDrive.setPower(0);
                }

                intakeState = true;

                if ((Math.abs(wristServoController.getCurrentPositionInDegrees() - 59) < 2) && (Math.abs(intakeArmServoController.getCurrentPositionInDegrees() - 77.7) < 2)) {
                    setPathState(21);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 21: // Transfer the piece
                deposClawState = true;
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.35)) {
                    break;
                }
                intakeClawState = false;
                setPathState(22);
                timeStamp = opmodeTimer.getElapsedTimeSeconds();
                break;
            case 22: // Start lifting up once piece is transferred
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.25)) {
                    break;
                }

                if (verticalRight.getCurrentPosition() < 3150) {
                    verticalLeft.setPower(-1);
                    verticalRight.setPower(1);
                } else {
                    setPathState(23);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 23: // Bring Depos Arm Up and Drop
                deposArmState = true;

                if (opmodeTimer.getElapsedTimeSeconds() > (timeStamp + 0.5)) { //(Math.abs(deposLeftController.getCurrentPositionInDegrees() - 85) < 2) {
                    deposClaw.setPosition(0.3);
                    setPathState(24);
                    timeStamp = opmodeTimer.getElapsedTimeSeconds();
                }
                break;
            case 24: // Go Park
                if (opmodeTimer.getElapsedTimeSeconds() < (timeStamp + 0.2)) {
                    break;
                }
                deposArmState = false;
                follower.followPath(park);
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        //region Magnetic Limit Switches
        magHorOn = !magLimHorizontal1.getState(); // Usually, "false" means pressed
        magVertOn = !magLimVertical1.getState(); // Usually, "false" means pressed
        limitSwitchNotTriggered = magLimVertical1.getState();
        //endregion

        if (intakeState) {
            moveWristTo("Close", intakeWrist);
            if (Math.abs(wristServoController.getCurrentPositionInDegrees() - 50) <= 10) {
                if (intakeWaitToReturn) {
                    //intakeClawState = true;
                    moveArmTo("Wait", intakeArm);
                } else {
                    moveArmTo("Close", intakeArm);
                }
            }
        } else {
            deposArmState = true;
            if (!grabbing) {
                moveWristTo("Open", intakeWrist);
            } else {
                moveWristTo("Grab", intakeWrist);
            }

            if ((Math.abs(wristServoController.getCurrentPositionInDegrees() - 9.16) <= 9.16) || (Math.abs(wristServoController.getCurrentPositionInDegrees() - 100) <= 10)) {
                if (!grabbing) {
                    //intakeClawState = false;
                    moveArmTo("Open", intakeArm);
                } else {
                    moveArmTo("Grab", intakeArm);
                }
            }
        }
        if (autoIntakeMode) {
            if (runtime.seconds() > grabTimer + 0.5 && grabbing) {
                grabbing = false;
            }
        } else {
            if (runtime.seconds() > grabTimer + 0.45 && grabbing) {
                grabbing = false;
            }
        }
        if (runtime.seconds() > grabTimer + 0.01 && grabbing && runtime.seconds() < grabTimer + 0.6) {
            intakeClawState = true;
        }

        if (intakeRotateState) { intakeRotate.setPosition(0); } else { intakeRotate.setPosition(1); }

        if (intakeClawState) { intakeClaw.setPosition(0); } else { intakeClaw.setPosition(1); }

        if (deposClawState) { deposClaw.setPosition(0.8); } else { deposClaw.setPosition(0.3); }

        if (deposArmState) {
            switch (scoreState) {
                case "Sample":
                    moveDeposTo("Depos", deposLeft, deposRight);
                    break;
                case "Specimen":
                    moveDeposTo("Specimen", deposLeft, deposRight);
                    break;
            }
        } else {
            moveDeposTo("Transfer", deposLeft, deposRight);
        }

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("depos arm value!", deposLeftController.getCurrentPositionInDegrees());
        telemetry.addData("claw pos!", deposClaw.getPosition());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        //region Find Hardware
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFront");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftBack");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBack");

        // All of the lift thingamajigs
        verticalRight = hardwareMap.get(DcMotor.class, "verticalRight");
        verticalLeft = hardwareMap.get(DcMotor.class, "verticalLeft");
        horizontalDrive = hardwareMap.get(DcMotor.class, "horizontalDrive");


        // All 4 input servos
        intakeClaw = hardwareMap.get(Servo.class, "intakeClaw"); // Exp. Hub P4
        intakeWrist = hardwareMap.get(Servo.class, "intakeWrist"); // Exp. Hub P3
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P2
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P1

        // All 3 output servos
        deposClaw = hardwareMap.get(Servo.class, "deposClaw");
        deposLeft = hardwareMap.get(Servo.class, "deposLeft");
        deposRight = hardwareMap.get(Servo.class, "deposRight");

        // All 3 special servo encoders
        depositEncoder1 = hardwareMap.get(AnalogInput.class, "depositEncoder1");
        depositEncoder2 = hardwareMap.get(AnalogInput.class, "depositEncoder2");
        wristEncoder1 = hardwareMap.get(AnalogInput.class, "wristEncoder1");
        wristEncoder2 = hardwareMap.get(AnalogInput.class, "wristEncoder2");
        armEncoder1 = hardwareMap.get(AnalogInput.class, "armEncoder1");
        armEncoder2 = hardwareMap.get(AnalogInput.class, "armEncoder2");


        // All the digital magnetic limit switches
        magLimVertical1 = hardwareMap.get(DigitalChannel.class, "magLimVertical1");
        magLimVertical2 = hardwareMap.get(DigitalChannel.class, "magLimVertical2");
        magLimHorizontal1 = hardwareMap.get(DigitalChannel.class, "magLimHorizontal1");
        magLimHorizontal2 = hardwareMap.get(DigitalChannel.class, "magLimHorizontal2");

        // Get sensors here
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        //telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(0);
        limelight.start();
        limelight.reloadPipeline();
        //endregion

        //region Configure Hardware
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        deposLeftController = new ContinuousServoController(dummy, depositEncoder1);
        deposRightController = new ContinuousServoController(dummy, depositEncoder1);
        wristServoController = new ContinuousServoController(dummy, wristEncoder1);
        intakeArmServoController = new ContinuousServoController(dummy, armEncoder1);

        verticalLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        magLimVertical1.setMode(DigitalChannel.Mode.INPUT);
        magLimHorizontal1.setMode(DigitalChannel.Mode.INPUT);

        //endregion
        horizontalDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    //region Servo Helper Functions
    // Move intake wrist to "Open" or "Close"
    public void moveArmTo(String state, Servo arm) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    arm.setPosition(0.4);
                } else {
                    arm.setPosition(0.55);
                }
                break;
            case "Close": // Equal to transfer position
                arm.setPosition(0);
                break;
            case "Grab": // Equal to grab position
                arm.setPosition(0.675);
                break;
            case "Wait": // Wait to return position
                arm.setPosition(0.4);
                break;
        }
    }

    // Move intake wrist to "Open" or "Close"
    public void moveWristTo(String state, Servo wrist) {
        switch (state) {
            case "Open": // Equal to grab position
                if (autoIntakeMode) {
                    wrist.setPosition(0);
                } else {
                    wrist.setPosition(0.225);
                }
                break;
            case "Close": // Equal to transfer position
                wrist.setPosition(1);
                break;
            case "Grab":
                wrist.setPosition(0.275);
                break;
        }
    }


    public void moveDeposTo(String state, Servo left, Servo right) {
        switch (state) {
            case "Transfer": // Equal to grab position
                left.setPosition(1);
                right.setPosition(0);
                break;
            case "Depos": // Equal to transfer position
                left.setPosition(0.54);
                right.setPosition(0.46);
                break;
            case "Specimen":
                left.setPosition(0.3);
                right.setPosition(0.7);
                break;
        }
    }
    //endregion

    //region State Finding Functions
    // Check if horizontal slides are all the way in
    public Boolean extendoClosed() { return (horizontalDrive.getCurrentPosition() < 25); }
    // Check if lift is all the way down
    public Boolean liftDown() { return (verticalRight.getCurrentPosition() < 25); }
    // Check if wrist and arm are back and claw is rotated in transfer position
    public Boolean intakeInTransferPosition(ContinuousServoController controllerWrist) { return (Math.abs(controllerWrist.getCurrentPositionInDegrees() - 60.5) < 5); }
    // Check if the depos arm is down
    public Boolean deposArmDown(ContinuousServoController controllerDepos) { return (Math.abs(controllerDepos.getCurrentPositionInDegrees() - 14) < 3); }
    // Check if the depos claw is closed
    public Boolean deposClawClosed() { return deposClawState; }

    // Get the state of the robot based on other values; can be overridden by certain controls
    public String getRobotState(ContinuousServoController controllerWrist, ContinuousServoController controllerDepos) {
        if (extendoClosed() && liftDown() && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && !deposClawClosed()) {
            // If everything retracted and depos claw open, basically starting position
            return "Transfer Ready";
        } else if (extendoClosed() && liftDown() && deposArmDown(controllerDepos) && intakeInTransferPosition(controllerWrist) && deposClawClosed()) {
            // If everything retracted and depos claw closed, basically starting position
            return "Transfer Complete";
        } else if (!extendoClosed() || !intakeInTransferPosition(controllerWrist)) {
            // If extendo not in and arm/wrist not retracted
            return "Grab";
        } else if (!liftDown() || !deposArmDown(controllerDepos)) {
            // If lift not down
            return "Depos";
        }
        return "Unknown";
    }
    //endregion
}
