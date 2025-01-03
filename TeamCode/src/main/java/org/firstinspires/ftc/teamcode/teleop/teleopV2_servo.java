/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Into the Deep: V2 TeleOp NEW", group="TeleOp")
public class teleopV2_servo extends LinearOpMode {

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
    private CRServo intakeArm =  null; // Edward
    private Servo intakeClaw =  null; // Servo that opens and closes intake claw
    private Servo intakeRotate =  null; // Servo that rotates the claw left right
    private CRServo intakeWrist =  null; // Servo that rotates the claw up down

    // All 3 of the Outtake Servos plugged into Control Hub
    private Servo deposClaw =  null; // Edward
    private CRServo deposLeft =  null; // Stuart
    private CRServo deposRight =  null; // Felicia

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

    private DistanceSensor backDistance = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
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
        intakeWrist = hardwareMap.get(CRServo.class, "intakeWrist"); // Exp. Hub P3
        intakeRotate = hardwareMap.get(Servo.class, "intakeRotate"); // Exp. Hub P2
        intakeArm = hardwareMap.get(CRServo.class, "intakeArm"); // Exp. Hub P1

        // All 3 output servos
        deposClaw = hardwareMap.get(Servo.class, "deposClaw");
        deposLeft = hardwareMap.get(CRServo.class, "deposLeft");
        deposRight = hardwareMap.get(CRServo.class, "deposRight");

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

        // The singular distance sensor we have. Yay.
        backDistance = hardwareMap.get(DistanceSensor.class, "backDistance");



        ContinuousServoController deposLeftController = new ContinuousServoController(deposLeft, depositEncoder1);
        ContinuousServoController deposRightController = new ContinuousServoController(deposRight, depositEncoder1);
        ContinuousServoController wristServoController = new ContinuousServoController(intakeWrist, wristEncoder1);
        ContinuousServoController intakeArmServoController = new ContinuousServoController(intakeArm, armEncoder1);


        // Servo Toggle Debounces
        Boolean intakeClawState = false; // Claw Open
        Boolean intakeRotateState = false; // Rotated in State 1
        Boolean intakeArmState = true; // Rotated in State 1
        Boolean intakeWristState = true; // Rotated in State 1
        Boolean deposClawState = true;
        Boolean deposArmState = true;

        Boolean intakeClawDebounce = false; // Claw Open
        Boolean intakeRotateDebounce = false; // Rotated in State 1
        Boolean intakeArmDebounce = false; // Rotated in State 1
        Boolean intakeWristDebounce = false; // Rotated in State 1
        Boolean deposClawDebounce = false;
        Boolean deposArmDebounce = false;

        Boolean horizontalDriveLockDebounce = false;
        Boolean horizontalDriveLockState = false;

        Boolean intakeState = false;
        Boolean intakeBoolean = false;

        double targetposition; // temp


        // ########################################################################################
        // !!!!            IMPORTANT Drive Information. Test your motor directions.            !!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double speedMultiplier = 0.67289;
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x;
            double yaw     =  gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double upDrivePower    = 0;
            double outDrivePower   = 0;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.
            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // LOCK THE OUT DRIVE!
            if (gamepad1.a && (!horizontalDriveLockDebounce)) {
                horizontalDriveLockDebounce = true;
                if (horizontalDriveLockState) {
                    // Lock The Out Drive
                    horizontalDriveLockState = false;
                    /*
                    outDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outDrive.setTargetPosition(outDrive.getCurrentPosition());
                     */
                } else {
                    // Unlock the Out Drive
                    horizontalDriveLockState = true;
                    horizontalDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    // (un)Rotate the servo
                }
            }
            if (!gamepad1.a && horizontalDriveLockDebounce) {
                horizontalDriveLockDebounce = false;
            }


            if (gamepad2.x && (!intakeClawDebounce)) {
                intakeClawDebounce = true;
                if (intakeClawState) {
                    intakeClawState = false;
                    intakeClaw.setPosition(0);
                } else {
                    intakeClawState = true;
                    intakeClaw.setPosition(1);
                }
            }
            if (!gamepad2.x && intakeClawDebounce) {
                intakeClawDebounce = false;
            }


            if (gamepad2.right_bumper && (!intakeRotateDebounce)) {
                intakeRotateDebounce = true;
                if (intakeRotateState) {
                    intakeRotateState = false;
                    intakeRotate.setPosition(0);
                } else {
                    intakeRotateState = true;
                    intakeRotate.setPosition(1);
                }
            }
            if (!gamepad2.right_bumper && intakeRotateDebounce) {
                intakeRotateDebounce = false;
            }



            if (gamepad2.y && (!deposClawDebounce)) {
                deposClawDebounce = true;
                if (deposClawState) {
                    deposClawState = false;
                    deposClaw.setPosition(0.3);
                } else {
                    deposClawState = true;
                    deposClaw.setPosition(0.8);
                }
            }
            if (!gamepad2.y && deposClawDebounce) {
                deposClawDebounce = false;
            }



            if (gamepad2.left_bumper && (!deposArmDebounce)) {
                deposArmDebounce = true;
                if (deposArmState) {
                    // Rotate the Wrist In
                    deposArmState = false;
                    //wristServoController.runToPosition(9.16, false);
                } else {
                    // Rotate the Wrist Out
                    deposArmState = true;
                    //wristServoController.runToPosition(64.8, true);
                }
            }
            if (!gamepad2.left_bumper && deposArmDebounce) {
                deposArmDebounce = false;
            }
            if (deposArmState) {
                if (deposLeftController.getCurrentPositionInDegrees() < 92 && deposLeftController.getCurrentPositionInDegrees() > 30) {
                    deposLeftController.runToPosition(92, false, 10);
                } else {
                    deposLeftController.runToPosition(92, true, 10);
                }
                //deposRightController.runToPosition(81, false, 1);
            } else {
                //deposRightController.runToPosition(14, true, 1);
                deposLeftController.runToPosition(14, false, 1);
            }
            //deposLeft.setPower(gamepad2.right_trigger - gamepad2.left_trigger);



            if (gamepad2.a && (!intakeWristDebounce)) {
                intakeWristDebounce = true;
                if (intakeWristState) {
                    // Rotate the Wrist In
                    intakeWristState = false;
                    //wristServoController.runToPosition(9.16, false);
                } else {
                    // Rotate the Wrist Out
                    intakeWristState = true;
                    //wristServoController.runToPosition(64.8, true);
                }
            }
            if (!gamepad2.a && intakeWristDebounce) {
                intakeWristDebounce = false;
            }
            if (intakeWristState) {
                if (wristServoController.getCurrentPositionInDegrees() < 58) {
                    wristServoController.runToPosition(58, true, 2.5);
                } else {
                    wristServoController.runToPosition(58, false, 2.5);
                }
            } else {
                if (wristServoController.getCurrentPositionInDegrees() > 9.16) {
                    wristServoController.runToPosition(9.16, false, 1);
                } else {
                    wristServoController.runToPosition(9.16, true, 1);
                }
            }


            if (gamepad2.b && (!intakeArmDebounce)) {
                intakeArmDebounce = true;
                if (intakeArmState) {
                    // Rotate the Arm In
                    intakeArmState = false;
                    //intakeArmServoController.runToPosition(76, false);
                } else {
                    // Rotate the Arm Out
                    intakeArmState = true;
                    //intakeArmServoController.runToPosition(50, true);
                }
            }
            if (!gamepad2.b && intakeArmDebounce) {
                intakeArmDebounce = false;
            }
            if (intakeArmState) {
                if (intakeArmServoController.getCurrentPositionInDegrees() < 77.7) {
                    intakeArmServoController.runToPosition(77.7, false, 1);
                } else {
                    intakeArmServoController.runToPosition(77.7, true, 1);
                }
            } else {
                if (intakeArmServoController.getCurrentPositionInDegrees() > 51.5) {
                    intakeArmServoController.runToPosition(51.5, true, 1);
                } else {
                    intakeArmServoController.runToPosition(51.5, false, 1);
                }
            }


            // Vertical Lift Motor Controls
            if (gamepad2.dpad_up) {
                upDrivePower = 1;
            } else if (gamepad2.dpad_down) {
                upDrivePower = -1;
            } else {
                upDrivePower = 0;
            }


            // Horizontal "Lift" Motor Controls
            if (gamepad2.dpad_right) {
                outDrivePower = 1;
            } else if (gamepad2.dpad_left) {
                outDrivePower = -1;
            } else {
                outDrivePower = 0;
            }


            if (gamepad1.left_bumper) {
                speedMultiplier = 0.25;
            } else if (gamepad1.right_bumper) {
                speedMultiplier = 1;
            }


            if ((horizontalDrive.getCurrentPosition() < -10) && (!gamepad2.dpad_right) && (!horizontalDriveLockState)) {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalDrive.setTargetPosition(0);
                outDrivePower = 0.2;
            } else if (horizontalDriveLockState) {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                horizontalDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                horizontalDrive.setTargetPosition(0);
                outDrivePower = 0.2;
            } else {
                horizontalDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower * speedMultiplier);
            rightFrontDrive.setPower(rightFrontPower * speedMultiplier);
            leftBackDrive.setPower(leftBackPower * speedMultiplier);
            rightBackDrive.setPower(rightBackPower * speedMultiplier);
            verticalRight.setPower(upDrivePower);
            verticalLeft.setPower(-upDrivePower);
            horizontalDrive.setPower(outDrivePower);

            int upDrivePos1 = verticalRight.getCurrentPosition();
            int upDrivePos2 = verticalLeft.getCurrentPosition();

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Lift Encoder Values: ", upDrivePos1 + ", " + upDrivePos2);

            telemetry.addData("Claw Debounce: ", intakeRotateDebounce);
            telemetry.addData("Claw State: ", intakeRotateState);
            //telemetry.addData("Depos Claw Position: ", intakeRotate.getPosition());

            telemetry.addData("Wrist Servo Encoder: ", (wristServoController.getCurrentPositionInDegrees()));
            telemetry.addData("Arm Servo Encoder: ", (intakeArmServoController.getCurrentPositionInDegrees()));
            telemetry.addData("Depos Servo Encoder: ", (deposLeftController.getCurrentPositionInDegrees()));

            telemetry.update();
        }
    }
}
