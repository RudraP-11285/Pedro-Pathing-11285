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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

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
abstract class ServoClass {
    private double[] positions;
    private boolean[] directions;
    public ServoClass(double[] positions, boolean[] directions) {
        this.positions = positions;
        this.directions = directions;
    }
    public double[] getPositions() {
        return positions;
    }
    public boolean[] getDirections() {
        return directions;
    }
}

class ContinuousServo extends ServoClass {
    private ContinuousServoController servo;
    private double[] tolerances;
    public ContinuousServo(double[] positions, boolean[] directions, double[] tolerances, ContinuousServoController servo) {
        super(positions,directions);
        this.servo = servo;
        this.tolerances = tolerances;
    }
    public double[] getTolerances() {
        return tolerances;
    }
    public ContinuousServoController getServo() {
        return servo;
    }
}
class StaticServo extends ServoClass {
    public StaticServo(double[] positions, boolean[] directions, double[] tolerances) {
        super(positions,directions);
    }
}


@TeleOp(name="Into the Deep: V2 TeleOp NEW^2", group="TeleOp")
public class teleopV2_servo2 extends LinearOpMode {
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

    public enum IntakeClawState {
        OPEN(1.0),
        INPROGRESS(0.0),
        CLOSE(0.0);
        private double state;
        IntakeClawState(double state) {
            this.state = state;
        }
        public double getState() {
            return state;
        }
    }

    public enum DeposClawState {
        OPEN(0.8),
        INPROGRESS(0.0),
        CLOSE(0.3);
        private double state;
        DeposClawState(double state) {
            this.state = state;
        }
        public double getState() {
            return state;
        }
    }
    DeposClawState deposClawState = DeposClawState.OPEN;
    IntakeClawState intakeClawState = IntakeClawState.OPEN;
    public enum ClawRotateState {
        VERITCAL(1.0),
        INPROGRESS(0.0),
        HORIZONTAL(0.0);
        private double state;
        ClawRotateState(double state) {
            this.state = state;
        }
        public double getState() {
            return state;
        }
    }
//    public enum DeposArmState {
//        UPC(92,true),
//        UPCC(92,false),
//        DOWNCC(14,false),
//        INPROGRESS(0,true);
//        private double pos;
//        private boolean clockwise;
//        DeposArmState(double pos, boolean clockwise) {
//            this.pos = pos;
//            this.clockwise = clockwise;
//        }
//        public double getPos() {
//            return pos;
//        }
//        public boolean isClockwise() {
//            return clockwise;
//        }
//    }
//    public enum IntakeWristState {
//        UPC(58,true),
//        UPCC(58,false),
//        DOWNC(9.16,true),
//        DOWNCC(9.16,false),
//        INPROGRESS(0,true);
//        private double pos;
//        private boolean clockwise;
//        IntakeWristState(double pos, boolean clockwise) {
//            this.pos = pos;
//            this.clockwise = clockwise;
//        }
//        public double getPos() {
//            return pos;
//        }
//        public boolean isClockwise() {
//            return clockwise;
//        }
//    }
//    IntakeWristState intakeWristState = IntakeWristState.DOWNC;
//    DeposArmState deposArmState = DeposArmState.DOWNCC;
//    public enum IntakeArmState {
//        UPC(77.7,true),
//        UPCC(77.7,false),
//        DOWNC(51.5,true),
//        DOWNCC(51.5,false),
//        INPROGRESS(0,true);
//        private double pos;
//        private boolean clockwise;
//        IntakeArmState(double pos, boolean clockwise) {
//            this.pos = pos;
//            this.clockwise = clockwise;
//        }
//        public double getPos() {
//            return pos;
//        }
//        public boolean isClockwise() {
//            return clockwise;
//        }
//    }
//    IntakeArmState intakeArmState = IntakeArmState.UPC;
    public enum ContinuousServoState {
        UPC(0),
        UPCC(1),
        DOWNC(2),
        DOWNCC(3),
        INPROGRESS(4),
        SPECIAL(5);
        private int index;
        ContinuousServoState(int index) {
            this.index = index;
        }
        public int getIndex() {
            return index;
        }
    }
    public enum ArmsStates {
        INTAKE,
        TRANSFER,
        DEPOS;
    }
    ArmsStates armsState = ArmsStates.INTAKE;
    /*
    public enum deposArmState {
        INPROGRESS(0)
        UPC(92)
        UPCC(92)
        DOWN(14)
    }
    */
//    public void toggleContinuousServo(ContinuousServoController servo, Enum<?> state) {
//        Class<? extends Enum> enumClass = state.getDeclaringClass();
//        state = enumClass.valueOf("UPC");
//
//    }
    //--------------------------------------------------------------------------------------------------------------------
    /*
    public void runServos(int startIndex, int endIndex, ContinuousServoState[] states, ContinuousServo[] values, ContinuousServoController[] servos) {
        for (int i = 0; i<3; i++) { // Iterates through all continuous servo instances in list ContinuousServoValues
            boolean gamepadControl = false;
            switch (i) {
                case 0:
                    gamepadControl = gamepad2.left_bumper;
                case 1:
                    gamepadControl = gamepad2.a;
                case 2:
                    gamepadControl = gamepad2.b;
            }
            ContinuousServoState currentState = states[i]; // Enumerator
            ContinuousServo currentServoVals = values[i]; // Class Instance
            ContinuousServoController currentServo = currentServoVals.getServo(); // Continuous Servo
            int j = 0; //Index of servo state for accessing properties of ServoClass
            int downPosIndex = ContinuousServoState.DOWNC.getIndex(); //Down position state index
            int upPosIndex = ContinuousServoState.UPC.getIndex(); // Up position state index
            double[] servoPositions = currentServoVals.getPositions();
            boolean[] servoDirections = currentServoVals.getDirections();
            double[] servoTolerances = currentServoVals.getTolerances();
            if (gamepadControl) {
                switch (states[i]) {
                    case UPC:
                    case UPCC:
                        states[i] = ContinuousServoState.INPROGRESS;
                        if (currentServo.getCurrentPositionInDegrees() > servoPositions[downPosIndex]) {
                            currentState = ContinuousServoState.DOWNC;
                        } else {
                            currentState = ContinuousServoState.DOWNCC;
                        }
                        j = currentState.getIndex();
                        currentServo.runToPosition(servoPositions[j],servoDirections[j],servoTolerances[j]);
                        states[i] = currentState;
                    case DOWNC:
                    case DOWNCC:
                        states[i] = ContinuousServoState.INPROGRESS;
                        if (currentServo == deposLeftController) {
                            if (currentServo.getCurrentPositionInDegrees() < servoPositions[ContinuousServoState.UPC.getIndex()] && currentServo.getCurrentPositionInDegrees() > 30) {
                                j = ContinuousServoState.UPCC.getIndex();
                            } else {
                                j = ContinuousServoState.UPC.getIndex();
                            }
                        } else {
                            if (currentServo.getCurrentPositionInDegrees() < servoPositions[upPosIndex]) {
                                if (currentServo == wristServoController) {
                                    currentState = ContinuousServoState.UPC;
                                } else if (currentServo == intakeArmServoController) {
                                    currentState = ContinuousServoState.UPCC;
                                }
                            } else {
                                if (currentServo == wristServoController) {
                                    currentState = ContinuousServoState.UPCC;
                                } else if (currentServo == intakeArmServoController) {
                                    currentState = ContinuousServoState.UPC;
                                }

                            }
                        }
                        j = currentState.getIndex();
                        currentServo.runToPosition(servoPositions[j],servoDirections[j],servoTolerances[j]);
                        states[i] = currentState;
                }
            }
        }
    }

     */
    ClawRotateState clawRotateState = ClawRotateState.HORIZONTAL;
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

        ContinuousServo deposArmValues = new ContinuousServo(new double[]{92,92,14,14}, new boolean[]{true,false,false,false}, new double[]{10,10,1,1},deposLeftController);
        ContinuousServo intakeWritstValues = new ContinuousServo(new double[]{58,58,9.16,9.16}, new boolean[]{true,false,true,false}, new double[]{2.5,2.5,1,1},wristServoController);
        ContinuousServo intakeArmValues = new ContinuousServo(new double[]{77.7,77.7,51.5,51.5}, new boolean[]{true,false,true,false}, new double[]{1,1,1,1},intakeArmServoController);
        ContinuousServo[] continuousServoValues = {deposArmValues,intakeWritstValues,intakeArmValues};

        ContinuousServoState deposArmState = ContinuousServoState.UPC;
        ContinuousServoState intakeWristState = ContinuousServoState.UPC;
        ContinuousServoState intakeArmState = ContinuousServoState.UPC;
        ContinuousServoState[] continuousServoStates = {deposArmState,intakeWristState,intakeArmState};
        ContinuousServoController[] continuousServos = {};

        // Servo Toggle Debounces
//        Boolean intakeClawState = false; // Claw Open
//        Boolean intakeRotateState = false; // Rotated in State 1
//        Boolean intakeArmState = true; // Rotated in State 1
//        Boolean intakeWristState = true; // Rotated in State 1
//        Boolean deposClawState = true;
//        Boolean deposArmState = true;

        boolean intakeClawDebounce = false; // Claw Open
        boolean intakeRotateDebounce = false; // Rotated in State 1
        boolean intakeArmDebounce = false; // Rotated in State 1
        boolean intakeWristDebounce = false; // Rotated in State 1
        boolean deposClawDebounce = false;
        boolean deposArmDebounce = false;

        boolean horizontalDriveLockDebounce = false;
        boolean horizontalDriveLockState = false;

        boolean intakeState = false;
        boolean intakeBoolean = false;

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

/*
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
*/
            //NEW CONDITIONALS
//            IntakeClawState tempClawState = IntakeClawState.OPEN;
            if (gamepad2.x) {
                switch (intakeClawState) {
                    case OPEN:
                        intakeClawState = IntakeClawState.INPROGRESS;
                        intakeClaw.setPosition(IntakeClawState.OPEN.getState());
//                        tempClawState = IntakeClawState.CLOSE;
                        break;
                    case CLOSE:
                        intakeClawState = IntakeClawState.INPROGRESS;
                        intakeClaw.setPosition(IntakeClawState.CLOSE.getState());
//                        tempClawState = IntakeClawState.OPEN;
                        break;
                }
            } else {
//                intakeClawState = tempClawState;
            }

            if (gamepad2.right_bumper) {
                switch (clawRotateState) {
                    case HORIZONTAL:
                        clawRotateState = ClawRotateState.INPROGRESS;
                        intakeRotate.setPosition(ClawRotateState.HORIZONTAL.getState());
                        clawRotateState = ClawRotateState.VERITCAL;
                        break;
                    case VERITCAL:
                        clawRotateState = ClawRotateState.INPROGRESS;
                        intakeRotate.setPosition(ClawRotateState.VERITCAL.getState());
                        clawRotateState = ClawRotateState.HORIZONTAL;
                        break;
                }
            }
            deposClawDebounce = false;
            if (gamepad2.y && !deposClawDebounce) {
                deposClawDebounce = true;
                switch (deposClawState) {
                    case OPEN:
                        deposClawState = DeposClawState.INPROGRESS;
                        deposClaw.setPosition(IntakeClawState.OPEN.getState());
                        deposClawState = DeposClawState.CLOSE;
                        break;
                    case CLOSE:
                        deposClawState = DeposClawState.INPROGRESS;
                        deposClaw.setPosition(DeposClawState.CLOSE.getState());
                        deposClawState = DeposClawState.OPEN;
                        break;
                }
            } else if (deposClawDebounce) {
                deposClawDebounce = false;
            }
            //----------------------------------------------------------original switch case statements-----------------------------------------------------
//            if (gamepad2.left_bumper) {
//                switch (deposArmState) {
//                    case UPC:
//                    case UPCC:
//                        deposArmState = DeposArmState.INPROGRESS;
//                        deposLeftController.runToPosition(DeposArmState.DOWNCC.getPos(),DeposArmState.DOWNCC.isClockwise(),1);
//                        deposArmState = DeposArmState.DOWNCC;
//                    case DOWNCC:
//                        deposArmState = DeposArmState.INPROGRESS;
//                        if (deposLeftController.getCurrentPositionInDegrees() < 92 && deposLeftController.getCurrentPositionInDegrees() > 30) {
//                            deposLeftController.runToPosition(DeposArmState.UPCC.getPos(),DeposArmState.UPCC.isClockwise(), 10);
//                            deposArmState = DeposArmState.UPCC;
//                        } else {
//                            deposLeftController.runToPosition(DeposArmState.UPC.getPos(),DeposArmState.UPC.isClockwise(),10);
//                            deposArmState = DeposArmState.UPC;
//                        }
//                }
//            }
//            if (gamepad2.a) {
//                switch (intakeWristState) {
//                    case UPC:
//                    case UPCC:
//                        intakeWristState = IntakeWristState.INPROGRESS;
//                        if (wristServoController.getCurrentPositionInDegrees() > 9.16) {
//                            wristServoController.runToPosition(IntakeWristState.DOWNCC.getPos(),IntakeWristState.DOWNCC.isClockwise(),1);
//                            intakeWristState = IntakeWristState.DOWNCC;
//                        } else {
//                            wristServoController.runToPosition(IntakeWristState.DOWNC.getPos(),IntakeWristState.DOWNC.isClockwise(),1);
//                            intakeWristState = IntakeWristState.DOWNC;
//                        }
//                    case DOWNC:
//                    case DOWNCC:
//                        intakeWristState = IntakeWristState.INPROGRESS;
//                        if (wristServoController.getCurrentPositionInDegrees() < 58) {
//                            wristServoController.runToPosition(IntakeWristState.UPC.getPos(), IntakeWristState.UPC.isClockwise(), 2.5);
//                            intakeWristState = IntakeWristState.UPC;
//                        } else {
//                            wristServoController.runToPosition(IntakeWristState.UPCC.getPos(), IntakeWristState.UPCC.isClockwise(), 2.5);
//                            intakeWristState = IntakeWristState.UPCC;
//                        }
//                }
//            }
//            if (gamepad2.b) {
//                switch (intakeArmState) {
//                    case UPC:
//                    case UPCC:
//                        intakeArmState = IntakeArmState.INPROGRESS;
//                        if (intakeArmServoController.getCurrentPositionInDegrees() > 51.5) {
//                            intakeArmServoController.runToPosition(IntakeArmState.DOWNC.getPos(),IntakeArmState.DOWNC.isClockwise(), 1);
//                            intakeArmState = IntakeArmState.DOWNC;
//                        } else {
//                            intakeArmServoController.runToPosition(IntakeArmState.DOWNCC.getPos(),IntakeArmState.DOWNCC.isClockwise(), 1);
//                            intakeArmState = IntakeArmState.DOWNCC;
//                        }
//                    case DOWNC:
//                    case DOWNCC:
//                        intakeArmState = IntakeArmState.INPROGRESS;
//                        if (intakeArmServoController.getCurrentPositionInDegrees() < 77.7) {
//                            intakeArmServoController.runToPosition(IntakeArmState.UPCC.getPos(),IntakeArmState.UPCC.isClockwise(), 1);
//                            intakeArmState = IntakeArmState.UPCC;
//                        } else {
//                            intakeArmServoController.runToPosition(IntakeArmState.UPC.getPos(),IntakeArmState.UPC.isClockwise(), 1);
//                            intakeArmState = IntakeArmState.UPC;
//                        }
//                }
//            }
            //---------------------------------------------------------------------------------------------------------------------------------------------
            for (int i = 0; i<3; i++) { // Iterates through all continuous servo instances in list ContinuousServoValues
                boolean gamepadControl = false;
                switch (i) {
                    case 0:
                        gamepadControl = gamepad2.left_bumper;
                        break;
                    case 1:
                        gamepadControl = gamepad2.a;
                        break;
                    case 2:
                        gamepadControl = gamepad2.b;
                        break;
                }
                ContinuousServoState currentState = continuousServoStates[i]; // Enumerator
                ContinuousServo currentServoVals = continuousServoValues[i]; // Class Instance
                ContinuousServoController currentServo = currentServoVals.getServo(); // Continuous Servo
                int j = 0; //Index of servo state for accessing properties of ServoClass
                int downPosIndex = ContinuousServoState.DOWNC.getIndex(); //Down position state index
                int upPosIndex = ContinuousServoState.UPC.getIndex(); // Up position state index
                double[] servoPositions = currentServoVals.getPositions();
                boolean[] servoDirections = currentServoVals.getDirections();
                double[] servoTolerances = currentServoVals.getTolerances();
                if (gamepadControl) {
                    switch (continuousServoStates[i]) {
                        case UPC:
                        case UPCC:
                            continuousServoStates[i] = ContinuousServoState.INPROGRESS;
                            if (currentServo.getCurrentPositionInDegrees() > servoPositions[downPosIndex]) {
                                if (currentServo == intakeWrist) {
                                    currentState = ContinuousServoState.DOWNCC;
                                } else {
                                    currentState = ContinuousServoState.DOWNC;
                                }

                            } else {
                                if (currentServo == intakeWrist) {
                                    currentState = ContinuousServoState.DOWNC;
                                } else {
                                    currentState = ContinuousServoState.DOWNCC;
                                }

                            }
                            j = currentState.getIndex();
                            currentServo.runToPosition(servoPositions[j],servoDirections[j],servoTolerances[j]);
                            continuousServoStates[i] = currentState;
                            break;
                        case DOWNC:
                        case DOWNCC:
                            continuousServoStates[i] = ContinuousServoState.INPROGRESS;
                            if (currentServo == deposLeftController) {
                                if (currentServo.getCurrentPositionInDegrees() < servoPositions[ContinuousServoState.UPC.getIndex()] && currentServo.getCurrentPositionInDegrees() > 30) {
                                    j = ContinuousServoState.UPCC.getIndex();
                                } else {
                                    j = ContinuousServoState.UPC.getIndex();
                                }
                            } else {
                                if (currentServo.getCurrentPositionInDegrees() < servoPositions[upPosIndex]) {
                                    if (currentServo == wristServoController) {
                                        currentState = ContinuousServoState.UPC;
                                    } else if (currentServo == intakeArmServoController) {
                                        currentState = ContinuousServoState.UPCC;
                                    }
                                } else {
                                    if (currentServo == wristServoController) {
                                        currentState = ContinuousServoState.UPCC;
                                    } else if (currentServo == intakeArmServoController) {
                                        currentState = ContinuousServoState.UPC;
                                    }

                                }
                            }
                            j = currentState.getIndex();
                            currentServo.runToPosition(servoPositions[j],servoDirections[j],servoTolerances[j]);
                            continuousServoStates[i] = currentState;
                            break;
                    }
                }
            }

            if (gamepad2.options) {
                switch (armsState) {
                    case INTAKE:
                        for (int i=0;i<2;i++) {
                            switch (continuousServoStates[i]) {

                            }
                        }
                    case TRANSFER:
                    case DEPOS:
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
//            telemetry.addData("Status", "Run Time: " + runtime.toString());
//            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
//            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
//            telemetry.addData("Lift Encoder Values: ", upDrivePos1 + ", " + upDrivePos2);
//
//            telemetry.addData("Claw Debounce: ", intakeRotateDebounce);
//            //telemetry.addData("Claw State: ", intakeRotateState);
//            //telemetry.addData("Depos Claw Position: ", intakeRotate.getPosition());
//
//            telemetry.addData("Wrist Servo Encoder: ", (wristServoController.getCurrentPositionInDegrees()));
//            telemetry.addData("Arm Servo Encoder: ", (intakeArmServoController.getCurrentPositionInDegrees()));
//            telemetry.addData("Depos Servo Encoder: ", (deposLeftController.getCurrentPositionInDegrees()));
            telemetry.update();
        }
    }
}
