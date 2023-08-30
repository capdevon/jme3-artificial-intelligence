package com.jme3.ai.navmesh.gen;

import java.beans.BeanInfo;
import java.beans.IntrospectionException;
import java.beans.Introspector;
import java.beans.PropertyDescriptor;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.Properties;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * 
 * @author capdevon
 */
public class NavMeshProperties {
    
    private static final Logger logger = Logger.getLogger(NavMeshProperties.class.getName());
    
    private NavMeshProperties() {}
    
    /**
     * Export the NavMeshBuildSettings to a file.
     * 
     * @param settings
     * @param file
     */
    public static void save(NavMeshBuildSettings settings, File file) {

        logger.log(Level.INFO, "Saving File={0}", file.getAbsolutePath());
        
        try (OutputStream output = new FileOutputStream(file)) {
            Properties prop = toProperties(settings);
            String comments = "NavMeshBuildSettings properties file";
            prop.store(output, comments);
            
        } catch (Exception ex) {
            logger.log(Level.SEVERE, "Error: Failed to save NavMeshBuildSettings!", ex);
        }
    }
    
    @SuppressWarnings("unchecked")
    private static <T> Properties toProperties(T t) 
            throws IntrospectionException, ReflectiveOperationException {

        Class<T> c = (Class<T>) t.getClass();
        BeanInfo beanInfo = Introspector.getBeanInfo(c, Object.class);
        Properties prop = new Properties();

        for (PropertyDescriptor pd : beanInfo.getPropertyDescriptors()) {
            String name = pd.getName();
            Object o = pd.getReadMethod().invoke(t);
            if (o != null) {
                prop.setProperty(name, o.toString());
            }
        }
        
        return prop;
    }
    
    /**
     * Reads a NavMeshBuildSettings from the input file.
     * 
     * @param file
     * @return
     * @throws IOException
     */
    public static NavMeshBuildSettings load(File file) throws IOException {
        try (FileInputStream input = new FileInputStream(file)) {
            return load(input);
        }
    }
    
    /**
     * Reads a NavMeshBuildSettings from the input byte stream.
     * 
     * @param input
     * @return
     * @throws IOException
     */
    public static NavMeshBuildSettings load(InputStream input) throws IOException {
    	
        Properties prop = new Properties();
        prop.load(input);

        NavMeshBuildSettings settings = new NavMeshBuildSettings();
        settings.cellSize                   = getFloat(prop, "cellSize");
        settings.cellHeight                 = getFloat(prop, "cellHeight");
        settings.minTraversableHeight       = getFloat(prop, "minTraversableHeight");
        settings.maxTraversableStep         = getFloat(prop, "maxTraversableStep");
        settings.maxTraversableSlope        = getFloat(prop, "maxTraversableSlope");
        settings.clipLedges                 = getBoolean(prop, "clipLedges");
        settings.traversableAreaBorderSize  = getFloat(prop, "traversableAreaBorderSize");
        settings.smoothingThreshold         = getInteger(prop, "smoothingThreshold");
        settings.useConservativeExpansion   = getBoolean(prop, "useConservativeExpansion");
        settings.minUnconnectedRegionSize   = getInteger(prop, "minUnconnectedRegionSize");
        settings.mergeRegionSize            = getInteger(prop, "mergeRegionSize");
        settings.maxEdgeLength              = getFloat(prop, "maxEdgeLength");
        settings.edgeMaxDeviation           = getFloat(prop, "edgeMaxDeviation");
        settings.maxVertsPerPoly            = getInteger(prop, "maxVertsPerPoly");
        settings.contourSampleDistance      = getFloat(prop, "contourSampleDistance");
        settings.contourMaxDeviation        = getFloat(prop, "contourMaxDeviation");
        
        return settings;
    }
    
    private static float getFloat(Properties prop, String key) {
        return Float.parseFloat(prop.getProperty(key));
    }
    
    private static int getInteger(Properties prop, String key) {
        return Integer.parseInt(prop.getProperty(key));
    }
    
    private static boolean getBoolean(Properties prop, String key) {
        return Boolean.parseBoolean(prop.getProperty(key));
    }
    
    private static String getString(Properties prop, String key) {
        return prop.getProperty(key);
    }
}
