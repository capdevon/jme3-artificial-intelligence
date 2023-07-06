package com.jme3.ai.test.util;

import java.util.logging.ConsoleHandler;
import java.util.logging.Handler;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.util.JmeFormatter;

/**
 * 
 * @author capdevon
 */
public class LogUtils {

    private LogUtils() {}

    /**
     * Set the log level specifying which message levels will be logged.
     * 
     * @param newLevel the new value for the log level (may be null) 
     */
    public static void setLevel(Level newLevel) {
        Logger root = Logger.getLogger("");
        root.setLevel(newLevel);
        for (Handler handler : root.getHandlers()) {
            handler.setLevel(newLevel);
        }
        System.out.println("level set: " + newLevel.getName());
    }
    
    public static void setJmeFormatter() {
        Logger rootLogger = Logger.getLogger("");
        for (Handler h : rootLogger.getHandlers()) {
            if (h instanceof ConsoleHandler) {
                ((ConsoleHandler) h).setFormatter(new JmeFormatter());
            }
        }
    }
}
