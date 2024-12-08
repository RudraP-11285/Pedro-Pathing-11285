package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class ContinuousServoController {
    private CRServo servo;
    private AnalogInput encoder; // Analog encoder
    private double maxEncoderValue = 100; // Max encoder value before it wraps around
    private double tolerance = 0.5; // Position tolerance (acceptable error)
    private double power = 0.1;

    public ContinuousServoController(CRServo servo, AnalogInput encoder) {
        this.servo = servo;
        this.encoder = encoder;
    }

    public void runToPosition(double targetPosition, boolean clockwise) {
        // Normalize the target position to within the encoder range
        targetPosition = normalizePosition(targetPosition);

        while (true) {
            // Get the current encoder position
            double currentPosition = encoder.getVoltage() * (maxEncoderValue / encoder.getMaxVoltage());
            currentPosition = normalizePosition(currentPosition);

            // Calculate the shortest direction to the target
            double error = calculateError(currentPosition, targetPosition);

            // Check if the target position is within the acceptable tolerance
            if (Math.abs(error) <= tolerance) {
                // Stop the servo
                servo.setPower(0);
                break;
            }

            // Determine servo direction
            if ((clockwise && error > 0) || (!clockwise && error < 0)) {
                servo.setPower(power); // Move clockwise
            } else {
                servo.setPower(power * -1); // Move counterclockwise
            }
        }
    }

    private double normalizePosition(double position) {
        if (position < 0) return position + maxEncoderValue;
        if (position >= maxEncoderValue) return position - maxEncoderValue;
        return position;
    }

    private double calculateError(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;

        // Adjust for wrap-around
        if (error > maxEncoderValue / 2) {
            error -= maxEncoderValue;
        } else if (error < -maxEncoderValue / 2) {
            error += maxEncoderValue;
        }

        return error;
    }
}
