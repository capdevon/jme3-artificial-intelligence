package com.jme3.ai.test.terrain;

import java.util.function.BiFunction;

import com.jme3.bounding.BoundingBox;
import com.jme3.bounding.BoundingSphere;
import com.jme3.bounding.BoundingVolume;
import com.jme3.math.Vector3f;
import com.jme3.renderer.Camera;
import com.jme3.renderer.Camera.FrustumIntersect;
import com.jme3.scene.Geometry;

/**
 * By default, it checks if geometry is in camera frustum and culls it if it is
 * outside camera view. A bound scale can be specified to scale geometry bound
 * before checking against camera frustum. This can be used to prevent shadow
 * disappearing when geometry gets slightly outside of camera frustum while it's
 * shadow is still visible.
 * 
 * @author Ali_RS
 */
public class MyInstanceCullingFunction implements BiFunction<Camera, Geometry, Boolean> {

    private Vector3f boundScale;
    private final BoundingBox tempBound = new BoundingBox();

    @Override
    public Boolean apply(Camera cam, Geometry geom) {
        BoundingVolume bv = geom.getWorldBound();
        if (boundScale != null) {
            if (bv instanceof BoundingBox) {
                bv = getScaledBound(boundScale, (BoundingBox) bv, tempBound);
            } else if (bv instanceof BoundingSphere) {
                bv = getScaledBound(boundScale, (BoundingSphere) bv, tempBound);
            }
        }

        int save = cam.getPlaneState();
        cam.setPlaneState(0);
        FrustumIntersect intersect = cam.contains(bv);
        cam.setPlaneState(save);

        return intersect == FrustumIntersect.Outside;
    }

    /**
     * Sets the scale to be applied on instance bound before checking against camera
     * frustum. Used to prevent shadow disappearing when geometry gets slightly
     * outside of camera frustum while it's shadow is still visible.
     *
     * (default is null)
     */
    public void setBoundScale(Vector3f boundScale) {
        this.boundScale = boundScale;
    }

    /**
     * @return the bound scale (default is null)
     */
    public Vector3f getBoundScale() {
        return boundScale;
    }

    protected BoundingBox getScaledBound(Vector3f scale, BoundingBox bound, BoundingBox store) {
        if (store == null) {
            store = new BoundingBox();
        }

        store.setCenter(bound.getCenter());
        store.setXExtent(bound.getXExtent() * scale.x);
        store.setYExtent(bound.getYExtent() * scale.y);
        store.setZExtent(bound.getZExtent() * scale.z);
        return store;
    }

    protected BoundingBox getScaledBound(Vector3f scale, BoundingSphere bound, BoundingBox store) {
        if (store == null) {
            store = new BoundingBox();
        }

        store.setCenter(bound.getCenter());
        store.setXExtent(bound.getRadius() * scale.x);
        store.setYExtent(bound.getRadius() * scale.y);
        store.setZExtent(bound.getRadius() * scale.z);
        return store;
    }
}
