package org.firstinspires.ftc.teamcode.subSystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class YArm {
    private final Motor SHOULDER_MOTOR;

    private final PIDController SHOULDER_PID = new PIDController(0.015, 0, 0.003);
    private double targetPosition = 0.0;

    public static double SHOULDER_INTAKING_POSITION = 0.0;
    public static double SHOULDER_OUTTAKING_POSITION = 1000.0;

    public YArm(HardwareMap hardwareMap) {

        SHOULDER_MOTOR = new Motor(hardwareMap, "shoulderMotor");

        SHOULDER_MOTOR.setRunMode(Motor.RunMode.RawPower);

        SHOULDER_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        SHOULDER_MOTOR.resetEncoder();
    }

    public void updatePID() {
        double currentPosition = SHOULDER_MOTOR.getCurrentPosition();

        double pidOutput = SHOULDER_PID.calculate(currentPosition, targetPosition);

        pidOutput = Math.max(-1.0, Math.min(1.0, pidOutput));

        SHOULDER_MOTOR.set(pidOutput);
    }


    public class MoveYarmTask extends Task {

        private final double TARGETPOSITION;

        public MoveYarmTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            TARGETPOSITION = targetPosition;
        }

        public void setShoulderTarget(double targetTicks) {
            SHOULDER_PID.setSetPoint(targetTicks);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            setShoulderTarget(TARGETPOSITION);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return Math.abs(SHOULDER_MOTOR.getCurrentPosition() - targetPosition) > 5;
        }
    }
}