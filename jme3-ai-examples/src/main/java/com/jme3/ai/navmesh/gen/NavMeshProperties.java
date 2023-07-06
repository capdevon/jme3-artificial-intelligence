package com.jme3.ai.navmesh.gen;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

public class NavMeshProperties {
    
    private NavMeshProperties() {}
    
    /**
     * 
     * @param file
     * @return
     * @throws IOException
     */
    public static NavMeshBuildSettings fromFile(File file) throws IOException {
    	
        try (FileInputStream fis = new FileInputStream(file)) {
            
            Properties props = new Properties();
            props.load(fis);

            NavMeshBuildSettings settings = new NavMeshBuildSettings();
            settings.cellSize                   = getFloat(props, "cellSize");
            settings.cellHeight                 = getFloat(props, "cellHeight");
            settings.minTraversableHeight       = getFloat(props, "minTraversableHeight");
            settings.maxTraversableStep         = getFloat(props, "maxTraversableStep");
            settings.maxTraversableSlope        = getFloat(props, "maxTraversableSlope");
            settings.clipLedges                 = getBoolean(props, "clipLedges");
            settings.traversableAreaBorderSize  = getFloat(props, "traversableAreaBorderSize");
            settings.smoothingThreshold         = getInteger(props, "smoothingThreshold");
            settings.useConservativeExpansion   = getBoolean(props, "useConservativeExpansion");
            settings.minUnconnectedRegionSize   = getInteger(props, "minUnconnectedRegionSize");
            settings.mergeRegionSize            = getInteger(props, "mergeRegionSize");
            settings.maxEdgeLength              = getFloat(props, "maxEdgeLength");
            settings.edgeMaxDeviation           = getFloat(props, "edgeMaxDeviation");
            settings.maxVertsPerPoly            = getInteger(props, "maxVertsPerPoly");
            settings.contourSampleDistance      = getFloat(props, "contourSampleDistance");
            settings.contourMaxDeviation        = getFloat(props, "contourMaxDeviation");
            
            return settings;
        }
    }
    
    private static float getFloat(Properties props, String key) {
        return Float.parseFloat(props.getProperty(key));
    }
    
    private static int getInteger(Properties props, String key) {
        return Integer.parseInt(props.getProperty(key));
    }
    
    private static boolean getBoolean(Properties props, String key) {
        return Boolean.parseBoolean(props.getProperty(key));
    }
    
    private static String getString(Properties props, String key) {
        return props.getProperty(key);
    }
}
