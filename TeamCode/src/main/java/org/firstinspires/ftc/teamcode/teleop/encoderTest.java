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
    int state = 0;
    boolean stateDebounce = false;

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P3
        signalA = hardwareMap.get(AnalogInput.class, "armEncoder1");
        signalB = hardwareMap.get(AnalogInput.class, "armEncoder2");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimVertical1"); // 'magLimVert1' is the name in the config file

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


            telemetry.addData("Depos Arm Encoder in Degrees:", (signalA.getVoltage() / signalA.getMaxVoltage()) * 360.0);
            telemetry.addData("State:", state);

            telemetry.update();
        }
    }
}
