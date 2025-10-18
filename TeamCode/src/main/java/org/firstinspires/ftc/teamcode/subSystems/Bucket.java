package org.firstinspires.ftc.teamcode.subSystems;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Bucket {
    public static final double BUCKET_MAX_POS = 1;
    public static final double BUCKET_MIN_POS = -1;

    private Servo bucketServo;

    public Bucket (HardwareMap hardwareMap) {
        this.bucketServo = bucketServo;
    }

    private double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    private void setBucketPos(double pos) {
        this.bucketServo.setPosition(limit(pos, BUCKET_MIN_POS, BUCKET_MAX_POS));
    }

    public class MoveBucketTask extends Task {
        private final double TARGET_POSITION;

        private final double estimatedTimeTaken;

        /**
         * Creates a new Task with the provided RobotContext.
         *
         * @param robotContext contains references like telemetry, gamepads, and subsystems
         */
        public MoveBucketTask(RobotContext robotContext, double targetPosition) {
            super(robotContext);
            TARGET_POSITION = targetPosition;
            this.estimatedTimeTaken = 5.0;
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            bucketServo.setPosition(TARGET_POSITION);


        }

        @Override
        protected boolean run(RobotContext robotContext) {
            return ELAPSED_TIME.seconds() < estimatedTimeTaken;
        }
    }

}



