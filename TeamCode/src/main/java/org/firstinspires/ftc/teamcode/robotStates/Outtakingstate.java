package org.firstinspires.ftc.teamcode.robotStates;

import com.jumpypants.murphy.RobotContext;
import com.jumpypants.murphy.states.State;
import com.jumpypants.murphy.tasks.ParallelTask;
import com.jumpypants.murphy.tasks.SequentialTask;
import com.jumpypants.murphy.tasks.Task;
import com.qualcomm.robotcore.hardware.Gamepad;




public class Outtakingstate implements State{

    private Task mainTask;

    @Override
    public State step() {
        return null;
    }

    @Override
    public String getName() {
        return "outtaking";
    }
}
