package com.jme3.ai.control;

import com.jme3.ai.navmesh.Path;
import com.jme3.ai.navmesh.Path.Waypoint;
import com.jme3.asset.AssetManager;
import com.jme3.cinematic.MotionPath;
import com.jme3.math.Spline;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Node;

/**
 *
 * @author capdevon
 */
public class PathViewer {

    // Asset manager
    protected AssetManager assetManager;

    // Node for attaching debug geometries
    private Node debugNode = new Node("Debug Node");
    private MotionPath motionPath;

    public PathViewer(AssetManager assetManager) {
        this.assetManager = assetManager;
        motionPath = new MotionPath();
        motionPath.setPathSplineType(Spline.SplineType.Linear);
    }

    /**
     * Displays a motion path showing each waypoint. 
     * Stays in scene until another path is set.
     *
     * @param path
     */
    public void drawPath(Path path) {
        clearPath();
        for (Waypoint wp : path.getWaypoints()) {
            motionPath.addWayPoint(wp.getPosition());
        }
        motionPath.enableDebugShape(assetManager, debugNode);
    }

    public void clearPath() {
        if (motionPath.getNbWayPoints() > 0) {
            motionPath.clearWayPoints();
            motionPath.disableDebugShape();
        }
    }

    /**
     * Render all the debug geometries to the specified view port.
     */
    public void show(RenderManager rm, ViewPort vp) {
        debugNode.updateLogicalState(0f);
        debugNode.updateGeometricState();
        rm.renderScene(debugNode, vp);
    }

}
