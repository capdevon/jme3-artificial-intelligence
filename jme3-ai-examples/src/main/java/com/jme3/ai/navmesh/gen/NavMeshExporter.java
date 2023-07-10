package com.jme3.ai.navmesh.gen;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;

import com.jme3.asset.AssetManager;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.export.binary.BinaryImporter;
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
    
    private AssetManager assetManager;

    public NavMeshExporter(AssetManager assetManager) {
        this.assetManager = assetManager;
    }
    
    /**
     * Saves the object into memory then loads it from memory.
     *
     * Used by tests to check if the persistence system is working.
     *
     * @param assetManager AssetManager to load assets from.
     * @param mesh The mesh to save and then load.
     * @return A new instance that has been saved and loaded from the
     * original mesh.
     */
    public static Geometry saveAndLoad(AssetManager assetManager, Mesh mesh) {
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        try {
            Geometry geo = makeGeometry(assetManager, mesh);

            BinaryExporter exporter = new BinaryExporter();
            exporter.save(geo, baos);

            BinaryImporter importer = new BinaryImporter();
            importer.setAssetManager(assetManager);
            return (Geometry) importer.load(baos.toByteArray());

        } catch (IOException ex) {
            // Should never happen.
            throw new AssertionError(ex);
        }
    }

    /**
     * Export the NavMesh to a file.
     * 
     * @param mesh The mesh to export
     * @param file The file to export to
     */ 
    public void save(Mesh mesh, File file) {
        Geometry geo = makeGeometry(assetManager, mesh);
        exportNavMesh(geo, file);
    }
    
    private static Geometry makeGeometry(AssetManager assetManager, Mesh mesh) {
        Geometry geo = new Geometry("NavMesh", mesh);
        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat.setColor("Color", ColorRGBA.Cyan);
        mat.getAdditionalRenderState().setWireframe(true);
        geo.setMaterial(mat);
        geo.setShadowMode(ShadowMode.Off);
        return geo;
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
