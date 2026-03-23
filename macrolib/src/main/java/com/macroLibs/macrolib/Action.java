package com.macroLibs.macrolib; // Make sure this matches your other files

public interface Action {
    void start();
    void update();
    boolean isFinished();
}