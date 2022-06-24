package com.z3db0y.susanalib;

import android.util.Log;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;

public class Logger {

    static private Telemetry telemetry;
    static private ArrayList<String> lines = new ArrayList<>();

    static public void setTelemetry(Telemetry telem) {
        telemetry = telem;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    static private String stripHTML(Object str) {
        return str.toString()
                .replaceAll("<", "&lt;")
                .replaceAll(">", "&gt;")
                .replaceAll(" ", "&nbsp;")
                .replaceAll("\"", "&quot;")
                .replaceAll("'", "&apos;");
    }

    static private void addDataInternal(Object message, String color, String caller) {
        if(color == null) color = "#ffffff";
        if(telemetry != null) {
            lines.add(caller + "() " + message);
            telemetry.addData("<span style='color: #ffaa00'>[SusanaLib] " + caller + "()</span>", "<span style='color: " + color + "'>" + stripHTML(message) + "</span>");
        }
    }

    static public void addData(Object message, String color) {
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        addDataInternal(message, color, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void addData(Object message) {
        StackTraceElement stackTraceElement = Thread.currentThread().getStackTrace()[3];
        addDataInternal(message, null, stackTraceElement.getClassName().split("\\.")[stackTraceElement.getClassName().split("\\.").length-1] + "." + stackTraceElement.getMethodName());
    }

    static public void update() {
        if(telemetry != null) {
            for(String line : lines) {
                Log.i("SusanaLib", line);
            }
            lines = new ArrayList<>();
            telemetry.update();
        }
    }

}
