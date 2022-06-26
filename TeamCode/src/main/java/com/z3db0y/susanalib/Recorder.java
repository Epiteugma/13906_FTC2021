package com.z3db0y.susanalib;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;
import java.util.HashMap;

public class Recorder {
    private HashMap<String, DcMotor> motors = new HashMap<>();
    private ArrayList<String> recording = new ArrayList<>();
    private Thread recordingThread;

    public void addMotor(String name, DcMotor motor) {
        motors.put(name, motor);
    }

    public void removeMotor(String name) {
        motors.remove(name);
    }

    public void start() {
        if(recordingThread != null) {
            recordingThread.interrupt();
        }
        recording.clear();
    }

}
