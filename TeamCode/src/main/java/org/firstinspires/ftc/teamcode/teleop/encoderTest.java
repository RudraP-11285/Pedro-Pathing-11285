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

    CRServo[] servos = new CRServo[7]; // Change the number based on how many servos you have
    int currentServoIndex = 0; // Keeps track of the current active servo

    @Override
    public void runOpMode() {
        // Initialize the two analog inputs
        servos[0] = hardwareMap.get(CRServo.class, "outtakeLeft");
        servos[1] = hardwareMap.get(CRServo.class, "outtakeRight");
        servos[2] = hardwareMap.get(CRServo.class, "intakeArm");
        servos[3] = hardwareMap.get(CRServo.class, "intakeClaw");
        servos[4] = hardwareMap.get(CRServo.class, "intakeRotate");
        servos[5] = hardwareMap.get(CRServo.class, "intakeWrist");
        servos[6] = hardwareMap.get(CRServo.class, "outputClaw");


        signalA = hardwareMap.get(AnalogInput.class, "outputClawEncoder1");
        signalB = hardwareMap.get(AnalogInput.class, "outputClawEncoder2");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "magLimVert1"); // 'magLimVert1' is the name in the config file
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                servos[currentServoIndex].setPower(0);
                sleep(500);
                currentServoIndex = (currentServoIndex + 1) % servos.length; // Cycle to the next servo
            }
            telemetry.addData("Current Servo Index", currentServoIndex);
            servos[currentServoIndex].setPower(gamepad1.right_trigger - gamepad1.left_trigger);


            // Read raw voltages from both wires
            double voltageA = signalA.getVoltage();
            double voltageB = signalB.getVoltage();
            boolean limitSwitchState = limitSwitch.getState();

            // Get the maximum voltage the Control Hub supports
            double maxVoltage = signalA.getMaxVoltage(); // Should be the same for both

            // Calculate the position from signal A
            double positionDegrees = (voltageA / maxVoltage) * 360;

            // Optional: Use signal B for additional information
            // Example: If signal B provides speed or direction
            double directionIndicator = voltageB; // Interpret as needed

            if (!limitSwitchState) {
                telemetry.addData("Limit Switch", "Triggered");
            } else {
                telemetry.addData("Limit Switch", "Not Triggered");
            }

            // Telemetry for debugging
            telemetry.addData("Signal A Voltage", voltageA);
            telemetry.addData("Signal B Voltage", voltageB);
            telemetry.addData("Position (degrees)", positionDegrees);
            telemetry.addData("Direction/Extra Info", directionIndicator);
            telemetry.update();
        }
    }
}
