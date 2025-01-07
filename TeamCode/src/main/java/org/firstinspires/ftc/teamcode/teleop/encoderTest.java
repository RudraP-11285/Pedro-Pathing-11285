package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Encoder Test", group="TeleOp")
public class encoderTest extends LinearOpMode {
    private AnalogInput signalA; // First wire
    private AnalogInput signalB; // Second wire
    private DigitalChannel limitSwitch;
    private Servo intakeArm =  null; // Servo that rotates the claw up down
    private DcMotor verticalDrive = null;
    int state = 0;
    boolean stateDebounce = false;

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P3
        signalA = hardwareMap.get(AnalogInput.class, "armEncoder1");
        signalB = hardwareMap.get(AnalogInput.class, "armEncoder2");
        verticalDrive = hardwareMap.get(DcMotor.class, "horizontalDrive");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimHorizontal1"); // 'magLimVert1' is the name in the config file
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        state = 0;
        stateDebounce = false;

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad2.a && !stateDebounce) {
                state += 1;
                state %= 3;
                stateDebounce = true;
            }
            if (stateDebounce && !gamepad2.a) {
                stateDebounce = false;
            }
            switch (state) {
                case 0:
                    intakeArm.setPosition(0);
                    break;
                case 1:
                    intakeArm.setPosition(0.55);
                    break;
                case 2:
                    intakeArm.setPosition(0.7);
                    break;
                default:
                    intakeArm.setPosition(0);
                    break;
            }

            boolean isPressed = !limitSwitch.getState(); // Usually, "false" means pressed

            // Display the state on the telemetry
            telemetry.addData("Limit Switch Pressed", isPressed);
            telemetry.addData("Horizontal Drive Position: ", verticalDrive.getCurrentPosition());
            telemetry.addData("Depos Arm Encoder in Degrees:", (signalA.getVoltage() / signalA.getMaxVoltage()) * 360.0);
            telemetry.addData("State:", state);

            telemetry.update();
        }
    }
}
