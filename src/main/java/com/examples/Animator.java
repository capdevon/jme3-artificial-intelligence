package com.examples;

import java.util.Objects;

import com.examples.util.GameObject;
import com.jme3.anim.AnimComposer;
import com.jme3.renderer.RenderManager;
import com.jme3.renderer.ViewPort;
import com.jme3.scene.Spatial;
import com.jme3.scene.control.AbstractControl;

/**
 * Spatial must have AnimComposer to use this control.
 *
 * @author capdevon
 */
public class Animator extends AbstractControl {

    private AnimComposer animComposer;
    private String currentAnim;

    @Override
    public void setSpatial(Spatial spatial) {
        super.setSpatial(spatial);

        if (spatial != null) {
            animComposer = GameObject.getComponentInChildren(spatial, AnimComposer.class);
            Objects.requireNonNull(animComposer, "AnimComposer not found: " + spatial);
        }
    }

    public void setAnimation(String animName) {
        if (!animName.equals(currentAnim)) {
            currentAnim = animName;
            animComposer.setCurrentAction(animName);
        }
    }

    public String getCurrentAnimName() {
        return currentAnim;
    }

    @Override
    protected void controlUpdate(float tpf) {
    }

    @Override
    protected void controlRender(RenderManager rm, ViewPort vp) {
    }
}
