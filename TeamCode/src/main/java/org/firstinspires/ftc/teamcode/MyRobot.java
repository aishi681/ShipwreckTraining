package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.jumpypants.murphy.RobotContext;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subSystems.Bucket;
import org.firstinspires.ftc.teamcode.subSystems.Wrist;
import org.firstinspires.ftc.teamcode.subSystems.Xarm;
import org.firstinspires.ftc.teamcode.subSystems.YArm;

/**
 * MyRobot class that extends RobotContext to include robot-specific subsystems.
 */
public class MyRobot extends RobotContext {
    public final MecanumDrive DRIVE;
    public final Xarm XARM;
    public final YArm YARM;
    public final Wrist WRIST;
    public final Bucket BUCKET;

    //not sure about the following
    public Gamepad gamepad1, gamepad2;

    /**
     * Creates a new RobotContext with the specified telemetry and gamepad references.
     * All parameters are required and cannot be null.
     *
     * @param telemetry the telemetry instance for driver station communication
     * @param gamepad1  the primary gamepad controller
     * @param gamepad2  the secondary gamepad controller
     */
    public MyRobot(Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2, MecanumDrive drive, Xarm xarm, YArm yarm, Wrist wrist, Bucket bucket, YArm YArm1, Wrist wrist1, Bucket bucket1) {
        super(telemetry, gamepad1, gamepad2);
        this.DRIVE = drive;
        this.XARM = xarm;
        this.YARM = YArm1;
        this.WRIST = wrist1;
        this.BUCKET = bucket1;

        //not sure about the following
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

    }
}