package com.jme3.ai.control;

import com.jme3.ai.navmesh.Path;
import com.jme3.ai.navmesh.Path.Waypoint;
import com.jme3.asset.AssetManager;
import com.jme3.environment.util.BoundingSphereDebug;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Spline;
import com.jme3.math.Spline.SplineType;
import com.jme3.math.Vector3f;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.control.AbstractControl;
import com.jme3.scene.shape.Curve;

/**
 *
 * @author capdevon
 */
public class PathViewer extends AbstractControl {

    // Asset manager
    private AssetManager assetManager;
    // Node for attaching debug geometries
    private Node debugNode = new Node("PathViewer");
    // Unshaded material
    private Material lineMat;
    private Material sphereMat;
    private BoundingSphereDebug sphere = new BoundingSphereDebug();
    private Spline spline = new Spline();
    private float pointSize = 0.2f;

    public PathViewer(AssetManager assetManager) {
        this.assetManager = assetManager;
        spline.setType(SplineType.Linear);
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
        for (Waypoint wp : path.getWaypoints()) {
            spline.addControlPoint(wp.getPosition());
            drawSphere(wp.getPosition(), pointSize);
        }
        drawCurve();
    }

    public void clearPath() {
        spline.clearControlPoints();
        debugNode.detachAllChildren();
    }
    
    @Override
    protected void controlUpdate(float tpf) {
    }
    
    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
        // Render all the debug geometries to the specified view port.
        debugNode.updateLogicalState(0f);
        debugNode.updateGeometricState();
        rm.renderScene(debugNode, vp);
    }
    
    private void drawCurve() {
        int nbSubSegments = 0;
        Geometry geo = new Geometry("PathCurve", new Curve(spline, nbSubSegments));
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
