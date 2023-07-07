package com.jme3.ai.navmesh.gen;

import java.io.File;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.asset.AssetManager;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Spatial;

/**
 * 
 * @author capdevon
 */
public class NavMeshExporter {

    private static final Logger logger = Logger.getLogger(NavMeshExporter.class.getName());
    
    protected AssetManager assetManager;

    public NavMeshExporter(AssetManager assetManager) {
        this.assetManager = assetManager;
    }
    
    /**
     * Export the NavMesh to a file.
     * 
     * @param mesh The mesh to export
     * @param file The file to export to
     */ 
    public void save(Mesh mesh, File file) {
        Geometry geo = new Geometry("NavMesh", mesh);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Cyan);
        mat.getAdditionalRenderState().setWireframe(true);
        geo.setMaterial(mat);
        geo.setShadowMode(ShadowMode.Off);
        exportNavMesh(geo, file);
    }

    private void exportNavMesh(Spatial sp, File file) {
        try {
            logger.log(Level.INFO, "Saving File={0}", file.getAbsolutePath());
            BinaryExporter exporter = BinaryExporter.getInstance();
            exporter.save(sp, file);

        } catch (IOException ex) {
            logger.log(Level.SEVERE, "Error: Failed to save NavMesh!", ex);
        }
    }

}
