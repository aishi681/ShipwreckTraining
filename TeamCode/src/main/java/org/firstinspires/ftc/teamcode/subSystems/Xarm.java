package org.firstinspires.ftc.teamcode.subSystems;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Xarm {

    private final Motor SLIDE_MOTOR;


    private final double KP = 0.1;
    private final double KI = 0.1;
    private final double KD = 0.1;
    private final PIDController PIDCONTROLLER = new PIDController(KP,KI,KD);
    public static final double MAX_POS = 1000.0;
    public static final double MIN_POS = 0.0;
    public static final double MAX_PWR = 1.0;
    public static final double MIN_PWR = -1.0;



    public Xarm(HardwareMap hardwareMap){
        SLIDE_MOTOR = new Motor(hardwareMap, "Xarm");
        SLIDE_MOTOR.setRunMode(Motor.RunMode.RawPower);
        SLIDE_MOTOR.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        SLIDE_MOTOR.resetEncoder();
    }

    public class MoveXarmTask extends Task {
        private final double TARGETPOSITION;
        public MoveXarmTask(RobotContext robotContext, double target_position){
            super(robotContext);
            TARGETPOSITION = Math.max(Math.min(MAX_POS,target_position), MIN_POS);
        }

        private void setPIDPoint(double target_position){
            PIDCONTROLLER.setSetPoint(target_position);
        }

        @Override
        protected void initialize(RobotContext robotContext) {
            setPIDPoint(TARGETPOSITION);
        }

        @Override
        protected boolean run(RobotContext robotContext) {
            double currPos = SLIDE_MOTOR.getCurrentPosition();
            double power = PIDCONTROLLER.calculate(currPos);
            power = Math.max(Math.min(power, MAX_PWR), MIN_PWR);
            SLIDE_MOTOR.set(power);
            return Math.abs(TARGETPOSITION - currPos) < 5;
        }
    }


}
