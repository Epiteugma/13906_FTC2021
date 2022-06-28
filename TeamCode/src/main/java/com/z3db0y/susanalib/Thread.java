package com.z3db0y.susanalib;

public class Thread {

    public static void sleep(int ms) {
        double start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < ms) {
            // do nothing
        }
    }

}
