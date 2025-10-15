package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YArm {
    private final Motor SLIDE_MOTOR;

    private final PIDController SLIDE_PID = new PIDController(0.015, 0, 0.003);
    private double targetPosition = 0.0;

    public static double SLIDE_INTAKING_POSITION = 0.0;
    public static double SLIDE_OUTTAKING_POSITION = 1000.0;

    public YArm(HardwareMap hardwareMap) {

        SLIDE_MOTOR = new Motor(hardwareMap, "slideMotor");

        SLIDE_MOTOR.setRunMode(Motor.RunMode.RawPower);

        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        SLIDE_MOTOR.resetEncoder();
    }

    public void updatePID() {
        double currentPosition = SLIDE_MOTOR.getCurrentPosition();

        double pidOutput = SLIDE_PID.calculate(currentPosition, targetPosition);

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        SLIDE_MOTOR.set(pidOutput);
    }


    public class MoveYarmTask extends Task {

        private final double TARGETPOSITION;

        public MoveYarmTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            TARGETPOSITION = targetPosition;
        }

        public void setSlideTarget(double targetTicks) {
            SLIDE_PID.setSetPoint(targetTicks);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            setSlideTarget(TARGETPOSITION);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return Math.abs(SLIDE_MOTOR.getCurrentPosition() - targetPosition) > 5;
        }
    }
}