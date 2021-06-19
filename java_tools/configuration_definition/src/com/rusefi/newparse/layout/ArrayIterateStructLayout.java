package com.rusefi.newparse.layout;

import com.rusefi.newparse.parsing.*;

import java.io.PrintStream;

public class ArrayIterateStructLayout extends ArrayLayout {
    public ArrayIterateStructLayout(StructField prototype, int length) {
        super(prototype, length);
    }

    private void emitOne(PrintStream ps, StructNamePrefixer prefixer, int offset, int idx) {
        // Set element's position within the array
        this.prototypeLayout.setOffset(offset + this.prototypeLayout.getSize() * idx);

        // Put a 1-based index on the end of the name to distinguish in TS
        prefixer.setSuffix(Integer.toString(idx + 1));
        this.prototypeLayout.writeTunerstudioLayout(ps, prefixer);
        prefixer.resetSuffix();
    }

    @Override
    public void writeTunerstudioLayout(PrintStream ps, StructNamePrefixer prefixer) {
        // Time to iterate: emit one scalar per array element, with the name modified accordingly

        for (int i = 0; i < this.length; i++) {
            emitOne(ps, prefixer, this.offset, i);
        }
    }

    // C layout is the same if iterated or not, use default implementation
}