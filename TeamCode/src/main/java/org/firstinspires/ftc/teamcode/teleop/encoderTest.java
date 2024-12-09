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
    private CRServo intakeArm =  null; // Servo that rotates the claw up down

    @Override
    public void runOpMode() {
        intakeArm = hardwareMap.get(CRServo.class, "intakeArm"); // Exp. Hub P3
        signalA = hardwareMap.get(AnalogInput.class, "armEncoder1");
        signalB = hardwareMap.get(AnalogInput.class, "armEncoder2");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimVertical1"); // 'magLimVert1' is the name in the config file
        waitForStart();

        ContinuousServoController controller = new ContinuousServoController(intakeArm, signalA);

        while (opModeIsActive()) {
            boolean limitSwitchState = limitSwitch.getState();
            if (!limitSwitchState) {
                telemetry.addData("Limit Switch", "Triggered");
            } else {
                telemetry.addData("Limit Switch", "Not Triggered");
            }

            telemetry.addData("Arm Encoder in Degrees", controller.getCurrentPositionInDegrees());

            telemetry.update();
        }
    }
}
