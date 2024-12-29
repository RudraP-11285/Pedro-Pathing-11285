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
    boolean state = false;
    boolean stateDebounce = false;

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(Servo.class, "intakeArm"); // Exp. Hub P3
        signalA = hardwareMap.get(AnalogInput.class, "depositEncoder1");
        signalB = hardwareMap.get(AnalogInput.class, "depositEncoder2");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimVertical1"); // 'magLimVert1' is the name in the config file

        state = false;
        stateDebounce = false;

        waitForStart();


        while (opModeIsActive()) {
            if (gamepad2.a && !stateDebounce) {
                state = !state;
            }
            if (stateDebounce && !gamepad2.a) {
                stateDebounce = false;
            }
            if (state) {
                intakeArm.setPosition(0);
            } else {
                intakeArm.setPosition(0.7);
            }

            telemetry.addData("Depos Arm Encoder in Degrees", (signalA.getVoltage() / signalA.getMaxVoltage()) * 360.0);

            telemetry.update();
        }
    }
}
