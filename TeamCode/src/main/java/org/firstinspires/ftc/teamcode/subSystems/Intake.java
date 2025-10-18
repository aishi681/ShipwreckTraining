package org.firstinspires.ftc.teamcode.subSystems;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.MyRobot;

//import org.firstinspires.ftc.teamcode.MyRobot;

public class Intake {

    private final Servo intakeLeftServo;
    private final Servo intakeRightServo;



    public Intake(HardwareMap hardwareMap) {
        intakeLeftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        intakeRightServo = hardwareMap.get(Servo.class, "intakeRightServo");
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
                    intakeLeftServo.setDirection(Servo.Direction.FORWARD);
                    intakeRightServo.setDirection(Servo.Direction.REVERSE);
                } else {
                    intakeLeftServo.setPosition(0.0);
                    intakeRightServo.setPosition(0.0);
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
                    intakeLeftServo.setDirection(Servo.Direction.REVERSE);
                    intakeRightServo.setDirection(Servo.Direction.FORWARD);
                } else {
                    intakeLeftServo.setPosition(0.0);
                    intakeRightServo.setPosition(0.0);
                }
            }

            previousButtonState = currentButtonState;

            return previousButtonState = false;
        }
    }


}


