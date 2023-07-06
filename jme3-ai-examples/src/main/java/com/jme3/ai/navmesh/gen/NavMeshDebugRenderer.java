package com.jme3.ai.navmesh.gen;

import com.jme3.asset.AssetManager;
import com.jme3.material.Material;
import com.jme3.material.RenderState.BlendMode;
import com.jme3.math.ColorRGBA;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.renderer.queue.RenderQueue.Bucket;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Mesh;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial.CullHint;

/**
 *
 * @author capdevon
 */
public class NavMeshDebugRenderer {

    // Asset manager
    protected AssetManager assetManager;
    // Node for attaching debug geometries
    public final Node debugNode = new Node("NavMeshDebugRenderer");

    /**
     *
     * @param assetManager
     */
    public NavMeshDebugRenderer(AssetManager assetManager) {
        this.assetManager = assetManager;
        debugNode.setCullHint(CullHint.Never);
    }

    public void drawNavMesh(Mesh mesh) {

        Geometry g1 = new Geometry("WireMesh", mesh);
        Material m1 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m1.setColor("Color", ColorRGBA.Black);
        m1.getAdditionalRenderState().setWireframe(true);
        g1.setMaterial(m1);
        g1.setShadowMode(ShadowMode.Off);
        debugNode.attachChild(g1);

        Geometry g2 = new Geometry("AlphaMesh", mesh);
        Material m2 = new Material(assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        m2.setColor("Color", ColorRGBA.Cyan.clone().setAlpha(0.1f));
        m2.getAdditionalRenderState().setBlendMode(BlendMode.Alpha);
        g2.setMaterial(m2);
        g2.setShadowMode(ShadowMode.Off);
        g2.setQueueBucket(Bucket.Transparent);
        debugNode.attachChild(g2);
    }

    public void clear() {
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

}
