package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Mechanism {
    void init(HardwareMap hardwareMap);
    void run(Gamepad gamepad);
}
