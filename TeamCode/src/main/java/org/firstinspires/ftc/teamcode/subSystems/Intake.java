package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.MyRobot;

public class Intake {

    private final CRServo intakeLeftServo;
    private final CRServo intakeRightServo;



    public Intake(HardwareMap hardwareMap) {
        intakeLeftServo = hardwareMap.get(CRServo.class, "intakeLeftServo");
        intakeRightServo = hardwareMap.get(CRServo.class, "intakeRightServo");
    }

    public class RunIntakeTask extends Task {
        private boolean previousButtonState = false;
        private boolean isServoOn = false;

        public RunIntakeTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            robotContext.telemetry.addLine("Starting intake control...");
            robotContext.telemetry.update();
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            Gamepad gamepad1 = robotContext.gamepad1;

            boolean currentButtonState = gamepad1.x;

            if (currentButtonState && !previousButtonState) {
                isServoOn = !isServoOn;

                if (isServoOn) {
                    intakeLeftServo.setPower(1.0);
                    intakeRightServo.setPower(1.0);
                } else {
                    intakeLeftServo.setPower(0.5);
                    intakeRightServo.setPower(0.5);
                }
            }

            previousButtonState = currentButtonState;

            return previousButtonState = false;
        }
    }
// if u click a button, then intake servo starts rolling and then stops
    //each task returns a boolean (if false, runs task again and if true, moves on to next)

    public class ReleaseSampleTask extends Task {
        private boolean previousButtonState = false;
        private boolean isServoOn = false;

        public ReleaseSampleTask(MyRobot robotContext) {
            super(robotContext);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            robotContext.telemetry.addLine("Starting intake control...");
            robotContext.telemetry.update();
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            Gamepad gamepad1 = robotContext.gamepad1;

            boolean currentButtonState = gamepad1.x;

            if (currentButtonState && !previousButtonState) {
                isServoOn = !isServoOn;

                if (isServoOn) {
                    intakeLeftServo.setPower(-1.0);
                    intakeRightServo.setPower(-1.0);

                } else {
                    intakeLeftServo.setPower(0.5);
                    intakeRightServo.setPower(0.5);
                }
            }

            previousButtonState = currentButtonState;

            return previousButtonState = false;
        }
    }


}


