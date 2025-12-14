package org.firstinspires.ftc.teamcode.flywheel;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "FlywheelPfTuner", group = "Tuning")
public class FlywheelPfTuner extends OpMode {

    public DcMotorEx flywheelMotor;

    // Target velocities (ticks/second or RPM depending on your units)
    public double highVelocity = 4200.0;
    public double lowVelocity  = 900.0;
    public double curTargetVelocity = highVelocity;

    // PF gains
    public double kP = 0.0;
    public double kF = 0.0;

    // Step sizes for tuning
    public double[] stepSizes = new double[] {10.0, 1.0, 0.1, 0.01, 0.001};
    public int stepIndex = 1;   // start at 1.0 steps

    // Edge-detected gamepad state handled by SDK (wasJustPressed-style in latest SDK)
    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        PIDFCoefficients pf = new PIDFCoefficients(kP, 0.0, 0.0, kF);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pf);

        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // Toggle between low and high velocity with Y
        if (gamepad1.y) {    // replace with wasJustPressed if using new input API
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {
                curTargetVelocity = highVelocity;
            }
        }

        // Cycle step size with B
        if (gamepad1.b) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        double step = stepSizes[stepIndex];

        // Tune F with D-pad left/right
        if (gamepad1.dpad_left) {
            kF -= step;
        }
        if (gamepad1.dpad_right) {
            kF += step;
        }

        // Tune P with D-pad up/down
        if (gamepad1.dpad_up) {
            kP += step;
        }
        if (gamepad1.dpad_down) {
            kP -= step;
        }

        // Apply new coefficients each loop
        PIDFCoefficients pf = new PIDFCoefficients(kP, 0.0, 0.0, kF);
        flywheelMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pf);

        // Command velocity
        flywheelMotor.setVelocity(curTargetVelocity);

        // Telemetry
        double curVelocity = flywheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target vel", curTargetVelocity);
        telemetry.addData("Actual vel", curVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("kP", kP);
        telemetry.addData("kF", kF);
        telemetry.addData("Step size", step);
        telemetry.update();
    }
}
