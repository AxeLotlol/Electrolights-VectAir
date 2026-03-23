package com.macroLibs.macrolib; // Make sure this matches your folder!

import java.util.function.BooleanSupplier; // You need this for isDone

public class MacroBuilder implements Action {
    private final Runnable start;
    private final BooleanSupplier isDone;

    public MacroBuilder(Runnable start, BooleanSupplier isDone) {
        this.start = start;
        this.isDone = isDone;
    }

    @Override
    public void start() {
        if (start != null) {
            start.run();
        }
    }

    @Override
    public void update() {
        // Simple actions usually don't need to do anything every frame
    }

    @Override
    public boolean isFinished() {
        return isDone.getAsBoolean();
    }
}