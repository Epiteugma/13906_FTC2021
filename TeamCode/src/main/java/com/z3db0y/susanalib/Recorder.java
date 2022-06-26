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
        if(recordingThread != null) recordingThread.interrupt();
        recording.clear();
        recordingThread = new Thread(() -> {
            while(!Thread.interrupted()) {
                double prevTime = 0;
                StringBuilder recLine = new StringBuilder();
                for(String name : motors.keySet()) {
                    recLine.append(name);
                    recLine.append("=");
                    recLine.append(motors.get(name).getCurrentPosition());
                    recLine.append("|");
                    recLine.append(motors.get(name).getPower());
                    recLine.append("&");
                }
                recLine.deleteCharAt(recLine.length() - 1);
                recording.add(recLine.toString());
                while(System.currentTimeMillis() < prevTime + 100) {}
            }
        });
        recordingThread.start();
    }

    public void stopAndSave(String filename) {
        if(recordingThread != null) recordingThread.interrupt();
    }

    public void replay(String filename) {}
}
