package com.jme3.ai.test;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.concurrent.TimeUnit;

import com.jme3.ai.navmesh.gen.GeometryProviderBuilder;
import com.jme3.ai.navmesh.gen.NavMeshBuildSettings;
import com.jme3.ai.navmesh.gen.NavMeshBuilder;
import com.jme3.ai.navmesh.gen.NavMeshDebugRenderer;
import com.jme3.ai.navmesh.gen.NavMeshProperties;
import com.jme3.app.SimpleApplication;
import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.system.AppSettings;

/**
 * 
 * @author capdevon
 */
public class Test_NavMesh extends SimpleApplication {

    /**
     * 
     * @param args
     */
    public static void main(String[] args) {
        Test_NavMesh app = new Test_NavMesh();
        AppSettings settings = new AppSettings(true);
        settings.setResolution(1280, 720);
        settings.setFrameRate(60);

        app.setSettings(settings);
        app.setShowSettings(false);
        app.setPauseOnLostFocus(false);
        app.start();
    }

    @Override
    public void simpleInitApp() {

        flyCam.setMoveSpeed(25);
        
        // Set the viewport's background color to light blue.
        ColorRGBA skyColor = new ColorRGBA(0.1f, 0.2f, 0.4f, 1f);
        viewPort.setBackgroundColor(skyColor);

        DirectionalLight light = new DirectionalLight();
        light.setDirection(new Vector3f(-0.3f, -0.5f, -0.2f));
        rootNode.addLight(light);

        AmbientLight amb = new AmbientLight();
        amb.setColor(ColorRGBA.DarkGray);
        rootNode.addLight(amb);

        Node scene = (Node) assetManager.loadModel("Models/navtest.j3o");

        NavMeshBuildSettings nmSettings = new NavMeshBuildSettings();
        try {
            File file = Path.of("src/main/resources", "Scenes", "navmesh.properties").toFile();
            nmSettings = NavMeshProperties.load(file);

        } catch (IOException ex) {
            ex.printStackTrace();
        }

        nmSettings.setCellSize(0.15f);
        nmSettings.setCellHeight(0.5f);
        nmSettings.setMinTraversableHeight(1.5f);
        nmSettings.setMaxTraversableStep(0.25f);
        nmSettings.setMaxTraversableSlope(48f);
        nmSettings.setClipLedges(true);
        nmSettings.setTraversableAreaBorderSize(0.1f);
        nmSettings.setSmoothingThreshold(2);
        nmSettings.setUseConservativeExpansion(true);
        nmSettings.setMinUnconnectedRegionSize(3);
        nmSettings.setMergeRegionSize(1);
        nmSettings.setMaxEdgeLength(0f);
        nmSettings.setEdgeMaxDeviation(0.25f);
        nmSettings.setMaxVertsPerPoly(6);
        nmSettings.setContourSampleDistance(100);
        nmSettings.setContourMaxDeviation(0.1f);

        List<Geometry> sources = GeometryProviderBuilder.collectSources(scene);
        sources.forEach(System.out::println);
        
        NavMeshBuilder builder = new NavMeshBuilder();
        builder.setTimeout(30, TimeUnit.SECONDS);
        
        Mesh navMesh = builder.buildNavMesh(sources, nmSettings);
        builder.shutdown();
        
        NavMeshDebugRenderer navMeshRenderer = new NavMeshDebugRenderer(assetManager);
        navMeshRenderer.drawNavMesh(navMesh);
        rootNode.attachChild(navMeshRenderer.debugNode);

//        Geometry navGeom = new Geometry("NavMesh", navMesh);
//        Material mat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
//        mat.getAdditionalRenderState().setWireframe(true);
//        mat.setColor("Color", ColorRGBA.Cyan);
//        navGeom.setMaterial(mat);
//        scene.attachChild(navGeom);

        rootNode.attachChild(scene);
    }

}