package com.jme3.ai.control;

import java.util.ArrayList;

import com.jme3.ai.navmesh.Path;
import com.jme3.ai.navmesh.Path.Waypoint;
import com.jme3.asset.AssetManager;
import com.jme3.environment.util.BoundingSphereDebug;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Line;

/**
 *
 * @author capdevon
 */
public class PathViewer {

    // Asset manager
    private AssetManager assetManager;
    // Node for attaching debug geometries
    private Node debugNode = new Node("PathViewer");
    // Unshaded material
    private Material lineMat;
    private Material sphereMat;
    private BoundingSphereDebug sphere = new BoundingSphereDebug();
    private float pointSize = 0.2f;

    public PathViewer(AssetManager assetManager) {
        this.assetManager = assetManager;
        setupMaterial();
    }
    
    /**
     * Initialize debug material
     */
    private void setupMaterial() {
        lineMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        lineMat.setColor("Color", ColorRGBA.Blue);
        
        sphereMat = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        sphereMat.setColor("Color", ColorRGBA.Red);
    }

    /**
     * Displays a motion path showing each waypoint. Stays in scene until
     * another path is set.
     *
     * @param path
     */
    public void drawPath(Path path) {
        clearPath();
        
        ArrayList<Waypoint> waypoints = path.getWaypoints();
        for (int j = 0; j < waypoints.size() - 1; j++) {
            Vector3f a = waypoints.get(j).getPosition();
            Vector3f b = waypoints.get(j + 1).getPosition();
            drawLine(a, b);
            drawSphere(a, pointSize);
        }
    }

    public void clearPath() {
        debugNode.detachAllChildren();
    }

    /**
     * Render all the debug geometries to the specified view port.
     */
    public void show(RenderManager rm, ViewPort vp) {
        debugNode.updateLogicalState(0f);
        debugNode.updateGeometricState();
        rm.renderScene(debugNode, vp);
    }
    
    private void drawLine(Vector3f start, Vector3f end) {
        Line line = new Line(start, end);
        Geometry geo = new Geometry("PathLine", line);
        geo.setMaterial(lineMat);
        debugNode.attachChild(geo);
    }

    private void drawSphere(Vector3f position, float radius) {
        Geometry geo = new Geometry("PathSphere", sphere);
        geo.setMaterial(sphereMat);
        geo.setLocalTranslation(position);
        geo.setLocalScale(radius);
        debugNode.attachChild(geo);
    }
    
    public float getPointSize() {
        return pointSize;
    }

    public void setPointSize(float pointSize) {
        this.pointSize = pointSize;
    }

}
